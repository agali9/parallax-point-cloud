#pragma once

#include <span>
#include <vector>

#include "pointcloud_pipeline/config.hpp"
#include "pointcloud_pipeline/types.hpp"
#include "pointcloud_pipeline/voxel_grid.hpp"

namespace pointcloud_pipeline {

std::vector<Cluster> euclideanCluster(std::span<const PointXYZ> cloud,
                                      const SegmentationConfig& config);

PointXYZ computeCentroid(std::span<const PointXYZ> cloud,
                         const std::vector<std::size_t>& indices);

BoundingBox computeBoundingBox(std::span<const PointXYZ> cloud,
                               const std::vector<std::size_t>& indices);

}  // namespace pointcloud_pipeline
