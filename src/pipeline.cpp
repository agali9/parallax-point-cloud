#include "pointcloud_pipeline/pipeline.hpp"

namespace pointcloud_pipeline {

PointCloudPipeline::PointCloudPipeline(PipelineConfig config) : config_(config) {}

PipelineResult PointCloudPipeline::process(const std::vector<PointXYZ>& cloud) const {
    (void)config_;
    PipelineResult result;
    result.filtered_cloud = cloud;
    return result;
}

}  // namespace pointcloud_pipeline