#include "pointcloud_pipeline/filtering.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>

#include "pointcloud_pipeline/voxel_grid.hpp"

namespace pointcloud_pipeline {
namespace {

bool isFinitePoint(const PointXYZ& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool isInsideBounds(const PointXYZ& point, const FilterConfig& config) {
    return point.x >= config.x_min && point.x <= config.x_max &&
           point.y >= config.y_min && point.y <= config.y_max &&
           point.z >= config.z_min && point.z <= config.z_max;
}

float squaredDistance(const PointXYZ& a, const PointXYZ& b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelKeyHash>
buildPointGrid(std::span<const PointXYZ> cloud, float cell_size) {
    std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelKeyHash> grid;
    grid.reserve(cloud.size());
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        grid[voxelIndex(cloud[i], cell_size)].push_back(i);
    }
    return grid;
}

std::vector<std::size_t> nearbyCandidateIndices(
    std::span<const PointXYZ> cloud,
    const std::unordered_map<VoxelKey, std::vector<std::size_t>, VoxelKeyHash>& grid,
    std::size_t point_index,
    float cell_size,
    int neighbor_count) {
    std::vector<std::size_t> candidates;
    candidates.reserve(static_cast<std::size_t>(neighbor_count) * 4U);

    const VoxelKey center = voxelIndex(cloud[point_index], cell_size);
    int ring = 0;
    while (candidates.size() < static_cast<std::size_t>(neighbor_count) + 1U && ring <= 4) {
        for (int dx = -ring; dx <= ring; ++dx) {
            for (int dy = -ring; dy <= ring; ++dy) {
                for (int dz = -ring; dz <= ring; ++dz) {
                    if (std::max({std::abs(dx), std::abs(dy), std::abs(dz)}) != ring) {
                        continue;
                    }
                    const VoxelKey key{center.x + dx, center.y + dy, center.z + dz};
                    const auto found = grid.find(key);
                    if (found == grid.end()) {
                        continue;
                    }
                    candidates.insert(candidates.end(), found->second.begin(), found->second.end());
                }
            }
        }
        ++ring;
    }

    if (candidates.size() < static_cast<std::size_t>(neighbor_count) + 1U) {
        candidates.resize(cloud.size());
        std::iota(candidates.begin(), candidates.end(), 0U);
    }

    return candidates;
}

float meanKNearestDistance(std::span<const PointXYZ> cloud,
                           const std::vector<std::size_t>& candidates,
                           std::size_t point_index,
                           int neighbor_count) {
    std::vector<float> distances;
    distances.reserve(candidates.size());

    for (const std::size_t candidate_index : candidates) {
        if (candidate_index == point_index) {
            continue;
        }
        distances.push_back(squaredDistance(cloud[point_index], cloud[candidate_index]));
    }

    if (distances.empty()) {
        return std::numeric_limits<float>::infinity();
    }

    const std::size_t k = std::min<std::size_t>(static_cast<std::size_t>(neighbor_count),
                                               distances.size());
    std::nth_element(distances.begin(), distances.begin() + static_cast<std::ptrdiff_t>(k - 1U),
                     distances.end());

    float sum = 0.0F;
    for (std::size_t i = 0; i < k; ++i) {
        sum += std::sqrt(distances[i]);
    }
    return sum / static_cast<float>(k);
}

}  // namespace

std::vector<PointXYZ> removeInvalidAndPassThrough(std::span<const PointXYZ> cloud,
                                                  const FilterConfig& config) {
    std::vector<PointXYZ> filtered;
    filtered.reserve(cloud.size());

    for (const PointXYZ& point : cloud) {
        if (isFinitePoint(point) && isInsideBounds(point, config)) {
            filtered.push_back(point);
        }
    }

    return filtered;
}

std::vector<PointXYZ> statisticalOutlierRemoval(std::span<const PointXYZ> cloud,
                                                int neighbor_count,
                                                float std_dev_multiplier,
                                                float grid_cell_size) {
    if (cloud.size() <= 2U || neighbor_count <= 0) {
        return std::vector<PointXYZ>(cloud.begin(), cloud.end());
    }

    const float safe_cell_size = std::max(grid_cell_size, 1.0e-3F);
    const int safe_neighbor_count = std::max(1, neighbor_count);
    const auto grid = buildPointGrid(cloud, safe_cell_size);

    std::vector<float> mean_distances(cloud.size(), 0.0F);
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        const std::vector<std::size_t> candidates =
            nearbyCandidateIndices(cloud, grid, i, safe_cell_size, safe_neighbor_count);
        mean_distances[i] = meanKNearestDistance(cloud, candidates, i, safe_neighbor_count);
    }

    const double sum = std::accumulate(mean_distances.begin(), mean_distances.end(), 0.0);
    const double mean = sum / static_cast<double>(mean_distances.size());

    double variance_sum = 0.0;
    for (const float distance : mean_distances) {
        const double delta = static_cast<double>(distance) - mean;
        variance_sum += delta * delta;
    }
    const double stddev = std::sqrt(variance_sum / static_cast<double>(mean_distances.size()));
    const double threshold = mean + static_cast<double>(std_dev_multiplier) * stddev;

    std::vector<PointXYZ> filtered;
    filtered.reserve(cloud.size());
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        if (static_cast<double>(mean_distances[i]) <= threshold) {
            filtered.push_back(cloud[i]);
        }
    }

    return filtered;
}

std::vector<PointXYZ> filterCloud(std::span<const PointXYZ> cloud,
                                  const FilterConfig& config) {
    std::vector<PointXYZ> filtered = removeInvalidAndPassThrough(cloud, config);
    if (!config.enable_statistical_outlier_removal) {
        return filtered;
    }
    return statisticalOutlierRemoval(filtered, config.neighbor_count,
                                     config.std_dev_multiplier,
                                     config.outlier_grid_cell_size);
}

}  // namespace pointcloud_pipeline
