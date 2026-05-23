#pragma once

#include <cstddef>

#include "pointcloud_pipeline/config.hpp"
#include "pointcloud_pipeline/cuda_backend.hpp"
#include "pointcloud_pipeline/types.hpp"

namespace pointcloud_pipeline::cuda_detail {

GpuPreprocessResult runGpuPreprocess(const PointXYZ* host_points,
                                     std::size_t point_count,
                                     const FilterConfig& filter,
                                     float voxel_size,
                                     bool enable_downsampling,
                                     bool filter_on_cpu);

}  // namespace pointcloud_pipeline::cuda_detail
