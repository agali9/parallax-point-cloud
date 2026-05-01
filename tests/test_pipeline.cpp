#include "test_common.hpp"

#include <vector>

#include "pointcloud_pipeline/pipeline.hpp"

using pointcloud_pipeline::PipelineConfig;
using pointcloud_pipeline::PointCloudPipeline;
using pointcloud_pipeline::PointXYZ;

TEST(Pipeline, RunsEndToEnd) {
    const std::vector<PointXYZ> cloud{{0.00F, 0.00F, 0.0F},
                                      {0.05F, 0.00F, 0.0F},
                                      {0.10F, 0.00F, 0.0F},
                                      {4.00F, 4.00F, 0.0F},
                                      {4.05F, 4.00F, 0.0F},
                                      {4.10F, 4.00F, 0.0F}};

    PipelineConfig config;
    config.filter.enable_statistical_outlier_removal = false;
    config.voxel.voxel_size = 0.02F;
    config.segmentation.cluster_tolerance = 0.2F;
    config.segmentation.min_cluster_size = 2U;

    const PointCloudPipeline pipeline(config);
    const auto result = pipeline.process(cloud);

    EXPECT_EQ(result.filtered_cloud.size(), 6U);
    EXPECT_EQ(result.downsampled_cloud.size(), 6U);
    EXPECT_EQ(result.clusters.size(), 2U);
    EXPECT_TRUE(result.timings.total_ms >= 0.0);
}
