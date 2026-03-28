#include "pointcloud_pipeline/pipeline.hpp"

#include "pointcloud_pipeline/filtering.hpp"
#include "pointcloud_pipeline/voxel_grid.hpp"

namespace pointcloud_pipeline {

PointCloudPipeline::PointCloudPipeline(PipelineConfig config) : config_(config) {}

PipelineResult PointCloudPipeline::process(const std::vector<PointXYZ>& cloud) const {
    PipelineResult result;
    result.filtered_cloud = filterCloud(cloud, config_.filter);
    result.downsampled_cloud = config_.enable_downsampling
        ? voxelGridDownsample(result.filtered_cloud, config_.voxel.voxel_size)
        : result.filtered_cloud;
    return result;
}

}  // namespace pointcloud_pipeline