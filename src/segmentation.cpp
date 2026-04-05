#include "pointcloud_pipeline/segmentation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace pointcloud_pipeline {
namespace {

float squaredDistance(const PointXYZ& a, const PointXYZ& b) {
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

}  // namespace

PointXYZ computeCentroid(const std::vector<PointXYZ>& cloud,
                         const std::vector<std::size_t>& indices) {
    PointXYZ centroid{};
    for (const std::size_t index : indices) {
        centroid.x += cloud[index].x;
        centroid.y += cloud[index].y;
        centroid.z += cloud[index].z;
    }
    const float count = static_cast<float>(indices.size());
    return PointXYZ{centroid.x / count, centroid.y / count, centroid.z / count};
}

BoundingBox computeBoundingBox(const std::vector<PointXYZ>& cloud,
                               const std::vector<std::size_t>& indices) {
    BoundingBox box{cloud[indices.front()], cloud[indices.front()]};
    for (const std::size_t index : indices) {
        const PointXYZ& point = cloud[index];
        box.min.x = std::min(box.min.x, point.x);
        box.min.y = std::min(box.min.y, point.y);
        box.min.z = std::min(box.min.z, point.z);
        box.max.x = std::max(box.max.x, point.x);
        box.max.y = std::max(box.max.y, point.y);
        box.max.z = std::max(box.max.z, point.z);
    }
    return box;
}

std::vector<Cluster> euclideanCluster(const std::vector<PointXYZ>& cloud,
                                      const SegmentationConfig& config) {
    std::vector<bool> visited(cloud.size(), false);
    std::vector<Cluster> clusters;
    const float tolerance_squared = config.cluster_tolerance * config.cluster_tolerance;

    for (std::size_t seed = 0; seed < cloud.size(); ++seed) {
        if (visited[seed]) {
            continue;
        }
        std::vector<std::size_t> indices;
        std::queue<std::size_t> frontier;
        frontier.push(seed);
        visited[seed] = true;
        while (!frontier.empty()) {
            const std::size_t current = frontier.front();
            frontier.pop();
            indices.push_back(current);
            for (std::size_t candidate = 0; candidate < cloud.size(); ++candidate) {
                if (!visited[candidate] && squaredDistance(cloud[current], cloud[candidate]) <= tolerance_squared) {
                    visited[candidate] = true;
                    frontier.push(candidate);
                }
            }
        }
        if (indices.size() >= config.min_cluster_size && indices.size() <= config.max_cluster_size) {
            Cluster cluster;
            cluster.id = static_cast<int>(clusters.size());
            cluster.indices = indices;
            cluster.centroid = computeCentroid(cloud, indices);
            cluster.bbox = computeBoundingBox(cloud, indices);
            clusters.push_back(cluster);
        }
    }
    return clusters;
}

}  // namespace pointcloud_pipeline