#include "minigtest.hpp"
#include "pointcloud_pipeline/segmentation.hpp"
#include "test_common.hpp"

#include <vector>

TEST(Segmentation, FindsTwoClusters) {
    std::vector<pointcloud_pipeline::PointXYZ> cloud{
        makePoint(0, 0, 0), makePoint(0.1F, 0, 0),
        makePoint(5, 5, 0), makePoint(5.1F, 5, 0)};
    pointcloud_pipeline::SegmentationConfig config;
    config.cluster_tolerance = 0.25F;
    config.min_cluster_size = 2;
    const auto clusters = pointcloud_pipeline::euclideanCluster(cloud, config);
    EXPECT_EQ(clusters.size(), 2U);
}