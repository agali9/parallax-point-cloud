#pragma once

#include <vector>

#include "pointcloud_pipeline/types.hpp"

namespace pointcloud_pipeline {

std::vector<PointXYZ> voxelGridDownsample(const std::vector<PointXYZ>& cloud,
                                          float voxel_size);

}  // namespace pointcloud_pipeline