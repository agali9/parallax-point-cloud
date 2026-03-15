#include <iostream>
#include <vector>

#include "pointcloud_pipeline/pipeline.hpp"

int main() {
    std::vector<pointcloud_pipeline::PointXYZ> cloud{{0.0F, 0.0F, 0.0F}, {1.0F, 0.0F, 0.0F}};
    const pointcloud_pipeline::PointCloudPipeline pipeline;
    const pointcloud_pipeline::PipelineResult result = pipeline.process(cloud);
    std::cout << "Loaded points: " << result.filtered_cloud.size() << '\n';
    return 0;
}