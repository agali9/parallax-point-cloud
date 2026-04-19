#include "pointcloud_pipeline/pipeline.hpp"

#include <chrono>

#include "pointcloud_pipeline/filtering.hpp"
#include "pointcloud_pipeline/segmentation.hpp"
#include "pointcloud_pipeline/voxel_grid.hpp"

namespace pointcloud_pipeline {
namespace {
using Clock = std::chrono::steady_clock;
double elapsedMs(Clock::time_point start, Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}
}  // namespace

PointCloudPipeline::PointCloudPipeline(PipelineConfig config) : config_(config) {}

PipelineResult PointCloudPipeline::process(const std::vector<PointXYZ>& cloud) const {
    PipelineResult result;
    const auto total_start = Clock::now();

    const auto filter_start = Clock::now();
    result.filtered_cloud = filterCloud(cloud, config_.filter);
    const auto filter_end = Clock::now();

    const auto downsample_start = Clock::now();
    result.downsampled_cloud = config_.enable_downsampling
        ? voxelGridDownsample(result.filtered_cloud, config_.voxel.voxel_size)
        : result.filtered_cloud;
    const auto downsample_end = Clock::now();

    const auto segmentation_start = Clock::now();
    result.clusters = euclideanCluster(result.downsampled_cloud, config_.segmentation);
    const auto segmentation_end = Clock::now();

    result.timings.filter_ms = elapsedMs(filter_start, filter_end);
    result.timings.downsample_ms = elapsedMs(downsample_start, downsample_end);
    result.timings.segmentation_ms = elapsedMs(segmentation_start, segmentation_end);
    result.timings.total_ms = elapsedMs(total_start, segmentation_end);
    return result;
}

}  // namespace pointcloud_pipeline