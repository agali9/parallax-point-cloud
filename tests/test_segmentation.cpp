#include "test_common.hpp"

#include <vector>

#include "pointcloud_pipeline/segmentation.hpp"

using pointcloud_pipeline::PointXYZ;
using pointcloud_pipeline::SegmentationConfig;
using pointcloud_pipeline::computeBoundingBox;
using pointcloud_pipeline::computeCentroid;
using pointcloud_pipeline::euclideanCluster;

TEST(Segmentation, DetectsTwoClusters) {
    const std::vector<PointXYZ> cloud{{0.0F, 0.0F, 0.0F},
                                      {0.1F, 0.0F, 0.0F},
                                      {0.0F, 0.1F, 0.0F},
                                      {5.0F, 5.0F, 0.0F},
                                      {5.1F, 5.0F, 0.0F},
                                      {5.0F, 5.1F, 0.0F}};
    SegmentationConfig config;
    config.cluster_tolerance = 0.25F;
    config.min_cluster_size = 2;

    const auto clusters = euclideanCluster(cloud, config);
    ASSERT_EQ(clusters.size(), 2U);
    EXPECT_EQ(clusters[0].indices.size(), 3U);
    EXPECT_EQ(clusters[1].indices.size(), 3U);
}

TEST(Segmentation, ComputesCentroid) {
    const std::vector<PointXYZ> cloud{{0.0F, 0.0F, 0.0F},
                                      {2.0F, 0.0F, 0.0F},
                                      {4.0F, 0.0F, 0.0F}};
    const std::vector<std::size_t> indices{0U, 1U, 2U};

    const PointXYZ centroid = computeCentroid(cloud, indices);
    EXPECT_NEAR(centroid.x, 2.0F, 1.0e-5F);
    EXPECT_NEAR(centroid.y, 0.0F, 1.0e-5F);
    EXPECT_NEAR(centroid.z, 0.0F, 1.0e-5F);
}

TEST(Segmentation, ComputesBoundingBox) {
    const std::vector<PointXYZ> cloud{{-1.0F, 2.0F, 0.0F},
                                      {2.0F, -3.0F, 1.0F},
                                      {0.0F, 4.0F, -2.0F}};
    const std::vector<std::size_t> indices{0U, 1U, 2U};

    const auto bbox = computeBoundingBox(cloud, indices);
    EXPECT_NEAR(bbox.min.x, -1.0F, 1.0e-5F);
    EXPECT_NEAR(bbox.min.y, -3.0F, 1.0e-5F);
    EXPECT_NEAR(bbox.min.z, -2.0F, 1.0e-5F);
    EXPECT_NEAR(bbox.max.x, 2.0F, 1.0e-5F);
    EXPECT_NEAR(bbox.max.y, 4.0F, 1.0e-5F);
    EXPECT_NEAR(bbox.max.z, 1.0F, 1.0e-5F);
}
