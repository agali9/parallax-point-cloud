#pragma once

#include <span>
#include <vector>

#include "pointcloud_pipeline/config.hpp"
#include "pointcloud_pipeline/types.hpp"

namespace pointcloud_pipeline {

class PointCloudPipeline {
public:
    PointCloudPipeline();
    explicit PointCloudPipeline(PipelineConfig config);

    [[nodiscard]] const PipelineConfig& config() const noexcept;
    void setConfig(PipelineConfig config);

    [[nodiscard]] PipelineResult process(std::span<const PointXYZ> cloud) const;
    [[nodiscard]] PipelineResult process(const std::vector<PointXYZ>& cloud) const;

private:
    PipelineConfig config_;
};

}  // namespace pointcloud_pipeline
