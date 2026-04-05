#pragma once

#include <vector>

#include "pointcloud_pipeline/config.hpp"
#include "pointcloud_pipeline/types.hpp"

namespace pointcloud_pipeline {

PointXYZ computeCentroid(const std::vector<PointXYZ>& cloud,
                         const std::vector<std::size_t>& indices);
BoundingBox computeBoundingBox(const std::vector<PointXYZ>& cloud,
                               const std::vector<std::size_t>& indices);
std::vector<Cluster> euclideanCluster(const std::vector<PointXYZ>& cloud,
                                      const SegmentationConfig& config);

}  // namespace pointcloud_pipeline