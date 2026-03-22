#pragma once

#include <vector>

#include "pointcloud_pipeline/config.hpp"
#include "pointcloud_pipeline/types.hpp"

namespace pointcloud_pipeline {

std::vector<PointXYZ> removeInvalidAndPassThrough(const std::vector<PointXYZ>& cloud,
                                                  const FilterConfig& config);

std::vector<PointXYZ> filterCloud(const std::vector<PointXYZ>& cloud,
                                  const FilterConfig& config);

}  // namespace pointcloud_pipeline