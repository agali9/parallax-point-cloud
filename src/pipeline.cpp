#include "pointcloud_pipeline/pipeline.hpp"

#include <chrono>
#include <utility>

#include "pointcloud_pipeline/filtering.hpp"
#include "pointcloud_pipeline/segmentation.hpp"
#include "pointcloud_pipeline/voxel_grid.hpp"

namespace pointcloud_pipeline {
namespace {

using Clock = std::chrono::steady_clock;

double elapsedMilliseconds(const Clock::time_point start, const Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
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
    PipelineResult result;

    const auto total_start = Clock::now();

    const auto filter_start = Clock::now();
    result.filtered_cloud = filterCloud(cloud, config_.filter);
    const auto filter_end = Clock::now();

    const auto downsample_start = Clock::now();
    if (config_.enable_downsampling) {
        result.downsampled_cloud =
            voxelGridDownsample(result.filtered_cloud, config_.voxel.voxel_size);
    } else {
        result.downsampled_cloud = result.filtered_cloud;
    }
    const auto downsample_end = Clock::now();

    const auto segmentation_start = Clock::now();
    result.clusters = euclideanCluster(result.downsampled_cloud, config_.segmentation);
    const auto segmentation_end = Clock::now();

    const auto total_end = Clock::now();

    result.timings.filter_ms = elapsedMilliseconds(filter_start, filter_end);
    result.timings.downsample_ms = elapsedMilliseconds(downsample_start, downsample_end);
    result.timings.segmentation_ms = elapsedMilliseconds(segmentation_start, segmentation_end);
    result.timings.total_ms = elapsedMilliseconds(total_start, total_end);

    return result;
}

PipelineResult PointCloudPipeline::process(const std::vector<PointXYZ>& cloud) const {
    return process(std::span<const PointXYZ>(cloud.data(), cloud.size()));
}

}  // namespace pointcloud_pipeline
