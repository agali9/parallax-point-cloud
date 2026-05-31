#pragma once

#include <span>
#include <vector>

#include "pointcloud_pipeline/config.hpp"
#include "pointcloud_pipeline/types.hpp"

namespace pointcloud_pipeline {

std::vector<PointXYZ> removeInvalidAndPassThrough(std::span<const PointXYZ> cloud,
                                                  const FilterConfig& config);

std::vector<PointXYZ> statisticalOutlierRemoval(std::span<const PointXYZ> cloud,
                                                int neighbor_count,
                                                float std_dev_multiplier,
                                                float grid_cell_size);

std::vector<PointXYZ> filterCloud(std::span<const PointXYZ> cloud,
                                  const FilterConfig& config);

}  // namespace pointcloud_pipeline
