#pragma once

#include <vector>

#include "pointcloud_pipeline/config.hpp"
#include "pointcloud_pipeline/types.hpp"

namespace pointcloud_pipeline {

struct PipelineResult {
    std::vector<PointXYZ> filtered_cloud;
    std::vector<PointXYZ> downsampled_cloud;
    std::vector<Cluster> clusters;
};

class PointCloudPipeline {
public:
    PointCloudPipeline() = default;
    explicit PointCloudPipeline(PipelineConfig config);

    [[nodiscard]] PipelineResult process(const std::vector<PointXYZ>& cloud) const;

private:
    PipelineConfig config_;
};

}  // namespace pointcloud_pipeline