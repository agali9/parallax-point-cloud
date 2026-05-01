#include <iostream>
#include <vector>

#include "pointcloud_pipeline/pipeline.hpp"

int main() {
    std::vector<pointcloud_pipeline::PointXYZ> cloud{
        {0.00F, 0.00F, 0.0F},
        {0.05F, 0.00F, 0.0F},
        {0.10F, 0.02F, 0.0F},
        {4.00F, 4.00F, 0.0F},
        {4.05F, 4.00F, 0.0F},
        {4.10F, 4.02F, 0.0F},
    };

    pointcloud_pipeline::PipelineConfig config;
    config.filter.enable_statistical_outlier_removal = false;
    config.voxel.voxel_size = 0.05F;
    config.segmentation.cluster_tolerance = 0.25F;
    config.segmentation.min_cluster_size = 2U;

    const pointcloud_pipeline::PointCloudPipeline pipeline(config);
    const pointcloud_pipeline::PipelineResult result = pipeline.process(cloud);

    std::cout << "Input points: " << cloud.size() << '\n';
    std::cout << "Filtered points: " << result.filtered_cloud.size() << '\n';
    std::cout << "Downsampled points: " << result.downsampled_cloud.size() << '\n';
    std::cout << "Clusters: " << result.clusters.size() << '\n';
    std::cout << "Total time: " << result.timings.total_ms << " ms\n";

    return 0;
}
