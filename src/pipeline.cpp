#include "pointcloud_pipeline/pipeline.hpp"

#include <chrono>
#include <utility>

#include "pointcloud_pipeline/cuda_backend.hpp"
#include "pointcloud_pipeline/filtering.hpp"
#include "pointcloud_pipeline/segmentation.hpp"
#include "pointcloud_pipeline/voxel_grid.hpp"

namespace pointcloud_pipeline {
namespace {

using Clock = std::chrono::steady_clock;

double elapsedMilliseconds(const Clock::time_point start, const Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

PipelineResult processOnCpu(std::span<const PointXYZ> cloud, const PipelineConfig& config) {
    PipelineResult result;

    const auto total_start = Clock::now();

    const auto filter_start = Clock::now();
    result.filtered_cloud = filterCloud(cloud, config.filter);
    const auto filter_end = Clock::now();

    const auto downsample_start = Clock::now();
    if (config.enable_downsampling) {
        result.downsampled_cloud =
            voxelGridDownsample(result.filtered_cloud, config.voxel.voxel_size);
    } else {
        result.downsampled_cloud = result.filtered_cloud;
    }
    const auto downsample_end = Clock::now();

    const auto segmentation_start = Clock::now();
    result.clusters = euclideanCluster(result.downsampled_cloud, config.segmentation);
    const auto segmentation_end = Clock::now();

    const auto total_end = Clock::now();

    result.timings.filter_ms = elapsedMilliseconds(filter_start, filter_end);
    result.timings.downsample_ms = elapsedMilliseconds(downsample_start, downsample_end);
    result.timings.segmentation_ms = elapsedMilliseconds(segmentation_start, segmentation_end);
    result.timings.total_ms = elapsedMilliseconds(total_start, total_end);
    result.timings.used_cuda = false;

    return result;
}

PipelineResult processWithCuda(std::span<const PointXYZ> cloud, const PipelineConfig& config) {
    PipelineResult result;
    const auto total_start = Clock::now();

    GpuPreprocessResult gpu_result;
    const auto filter_start = Clock::now();

    if (config.filter.enable_statistical_outlier_removal) {
        result.filtered_cloud = filterCloud(cloud, config.filter);
        gpu_result = gpuPreprocess(result.filtered_cloud, config.filter, config.voxel.voxel_size,
                                   config.enable_downsampling, true);
    } else {
        gpu_result =
            gpuPreprocess(cloud, config.filter, config.voxel.voxel_size,
                          config.enable_downsampling, false);
        result.filtered_cloud = std::move(gpu_result.filtered_cloud);
    }

    const auto filter_end = Clock::now();
    result.downsampled_cloud = std::move(gpu_result.downsampled_cloud);

    const auto segmentation_start = Clock::now();
    result.clusters = euclideanCluster(result.downsampled_cloud, config.segmentation);
    const auto segmentation_end = Clock::now();
    const auto total_end = Clock::now();

    result.timings.used_cuda = true;
    result.timings.h2d_ms = gpu_result.timings.h2d_ms;
    result.timings.d2h_ms = gpu_result.timings.d2h_ms;
    if (config.filter.enable_statistical_outlier_removal) {
        result.timings.filter_ms = elapsedMilliseconds(filter_start, filter_end);
        result.timings.downsample_ms = gpu_result.timings.voxel_ms;
    } else {
        result.timings.filter_ms = gpu_result.timings.filter_ms;
        result.timings.downsample_ms = gpu_result.timings.voxel_ms;
    }
    result.timings.segmentation_ms = elapsedMilliseconds(segmentation_start, segmentation_end);
    result.timings.total_ms = elapsedMilliseconds(total_start, total_end);

    return result;
}

}  // namespace

PointCloudPipeline::PointCloudPipeline() = default;

PointCloudPipeline::PointCloudPipeline(PipelineConfig config) : config_(std::move(config)) {}

const PipelineConfig& PointCloudPipeline::config() const noexcept {
    return config_;
}

void PointCloudPipeline::setConfig(PipelineConfig config) {
    config_ = std::move(config);
}

PipelineResult PointCloudPipeline::process(std::span<const PointXYZ> cloud) const {
    if (shouldUseCuda(config_)) {
        return processWithCuda(cloud, config_);
    }
    return processOnCpu(cloud, config_);
}

PipelineResult PointCloudPipeline::process(const std::vector<PointXYZ>& cloud) const {
    return process(std::span<const PointXYZ>(cloud.data(), cloud.size()));
}

}  // namespace pointcloud_pipeline
