#pragma once

#include <cstddef>
#include <span>
#include <vector>

#include "pointcloud_pipeline/config.hpp"
#include "pointcloud_pipeline/types.hpp"

namespace pointcloud_pipeline {

[[nodiscard]] bool isCudaAvailable() noexcept;
[[nodiscard]] const char* cudaDeviceName() noexcept;

struct GpuStageTimings {
    double h2d_ms = 0.0;
    double filter_ms = 0.0;
    double voxel_ms = 0.0;
    double d2h_ms = 0.0;
};

struct GpuPreprocessResult {
    std::vector<PointXYZ> filtered_cloud;
    std::vector<PointXYZ> downsampled_cloud;
    GpuStageTimings timings;
};

[[nodiscard]] bool shouldUseCuda(const PipelineConfig& config) noexcept;

[[nodiscard]] GpuPreprocessResult gpuPreprocess(std::span<const PointXYZ> cloud,
                                                const FilterConfig& filter,
                                                float voxel_size,
                                                bool enable_downsampling,
                                                bool filter_on_cpu);

}  // namespace pointcloud_pipeline
