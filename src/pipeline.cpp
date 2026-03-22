#include "pointcloud_pipeline/pipeline.hpp"

#include "pointcloud_pipeline/filtering.hpp"

namespace pointcloud_pipeline {

PointCloudPipeline::PointCloudPipeline(PipelineConfig config) : config_(config) {}

PipelineResult PointCloudPipeline::process(const std::vector<PointXYZ>& cloud) const {
    PipelineResult result;
    result.filtered_cloud = filterCloud(cloud, config_.filter);
    return result;
}

}  // namespace pointcloud_pipeline