#include "pointcloud_pipeline/segmentation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>

namespace pointcloud_pipeline {
namespace {

float squaredDistance(const PointXYZ& a, const PointXYZ& b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelKeyHash>
buildSearchGrid(std::span<const PointXYZ> cloud, float cell_size) {
    std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelKeyHash> grid;
    grid.reserve(cloud.size());
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        grid[voxelIndex(cloud[i], cell_size)].push_back(i);
    }
    return grid;
}

}  // namespace

PointXYZ computeCentroid(std::span<const PointXYZ> cloud,
                         const std::vector<std::size_t>& indices) {
    if (indices.empty()) {
        return PointXYZ{0.0F, 0.0F, 0.0F};
    }

    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    for (const std::size_t index : indices) {
        sum_x += cloud[index].x;
        sum_y += cloud[index].y;
        sum_z += cloud[index].z;
    }

    const double count = static_cast<double>(indices.size());
    return PointXYZ{static_cast<float>(sum_x / count),
                    static_cast<float>(sum_y / count),
                    static_cast<float>(sum_z / count)};
}

BoundingBox computeBoundingBox(std::span<const PointXYZ> cloud,
                               const std::vector<std::size_t>& indices) {
    if (indices.empty()) {
        return BoundingBox{PointXYZ{0.0F, 0.0F, 0.0F}, PointXYZ{0.0F, 0.0F, 0.0F}};
    }

    PointXYZ min_point{std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max(),
                       std::numeric_limits<float>::max()};
    PointXYZ max_point{std::numeric_limits<float>::lowest(),
                       std::numeric_limits<float>::lowest(),
                       std::numeric_limits<float>::lowest()};

    for (const std::size_t index : indices) {
        const PointXYZ& point = cloud[index];
        min_point.x = std::min(min_point.x, point.x);
        min_point.y = std::min(min_point.y, point.y);
        min_point.z = std::min(min_point.z, point.z);
        max_point.x = std::max(max_point.x, point.x);
        max_point.y = std::max(max_point.y, point.y);
        max_point.z = std::max(max_point.z, point.z);
    }

    return BoundingBox{min_point, max_point};
}

std::vector<Cluster> euclideanCluster(std::span<const PointXYZ> cloud,
                                      const SegmentationConfig& config) {
    if (config.cluster_tolerance <= 0.0F) {
        throw std::invalid_argument("cluster_tolerance must be positive");
    }
    if (cloud.empty()) {
        return {};
    }

    const float tolerance_squared = config.cluster_tolerance * config.cluster_tolerance;
    const float cell_size = config.cluster_tolerance;
    const auto grid = buildSearchGrid(cloud, cell_size);

    std::vector<bool> visited(cloud.size(), false);
    std::vector<Cluster> clusters;
    int next_cluster_id = 0;

    for (std::size_t seed = 0; seed < cloud.size(); ++seed) {
        if (visited[seed]) {
            continue;
        }

        std::vector<std::size_t> cluster_indices;
        std::queue<std::size_t> frontier;
        visited[seed] = true;
        frontier.push(seed);

        while (!frontier.empty()) {
            const std::size_t current = frontier.front();
            frontier.pop();
            cluster_indices.push_back(current);

            const VoxelKey center = voxelIndex(cloud[current], cell_size);
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        const VoxelKey key{center.x + dx, center.y + dy, center.z + dz};
                        const auto found = grid.find(key);
                        if (found == grid.end()) {
                            continue;
                        }
                        for (const std::size_t neighbor : found->second) {
                            if (visited[neighbor]) {
                                continue;
                            }
                            if (squaredDistance(cloud[current], cloud[neighbor]) <=
                                tolerance_squared) {
                                visited[neighbor] = true;
                                frontier.push(neighbor);
                            }
                        }
                    }
                }
            }
        }

        std::sort(cluster_indices.begin(), cluster_indices.end());
        if (cluster_indices.size() >= config.min_cluster_size &&
            cluster_indices.size() <= config.max_cluster_size) {
            Cluster cluster;
            cluster.id = next_cluster_id++;
            cluster.indices = std::move(cluster_indices);
            cluster.centroid = computeCentroid(cloud, cluster.indices);
            cluster.bbox = computeBoundingBox(cloud, cluster.indices);
            clusters.push_back(std::move(cluster));
        }
    }

    return clusters;
}

}  // namespace pointcloud_pipeline
