#include "test_common.hpp"

#include <cmath>
#include <limits>
#include <vector>

#include "pointcloud_pipeline/filtering.hpp"

using pointcloud_pipeline::FilterConfig;
using pointcloud_pipeline::PointXYZ;
using pointcloud_pipeline::filterCloud;
using pointcloud_pipeline::removeInvalidAndPassThrough;
using pointcloud_pipeline::statisticalOutlierRemoval;

TEST(Filtering, RemovesNaN) {
    const std::vector<PointXYZ> cloud{{0.0F, 0.0F, 0.0F},
                                      {std::numeric_limits<float>::quiet_NaN(), 1.0F, 2.0F},
                                      {1.0F, 1.0F, 1.0F}};
    FilterConfig config;
    config.enable_statistical_outlier_removal = false;

    const auto filtered = filterCloud(cloud, config);
    ASSERT_EQ(filtered.size(), 2U);
    EXPECT_NEAR(filtered[0].x, 0.0F, 1.0e-5F);
    EXPECT_NEAR(filtered[1].x, 1.0F, 1.0e-5F);
}

TEST(Filtering, RemovesInfinity) {
    const std::vector<PointXYZ> cloud{{0.0F, 0.0F, 0.0F},
                                      {1.0F, std::numeric_limits<float>::infinity(), 2.0F},
                                      {1.0F, 1.0F, 1.0F}};
    FilterConfig config;
    config.enable_statistical_outlier_removal = false;

    const auto filtered = filterCloud(cloud, config);
    ASSERT_EQ(filtered.size(), 2U);
    EXPECT_TRUE(std::isfinite(filtered[0].x));
    EXPECT_TRUE(std::isfinite(filtered[1].x));
}

TEST(Filtering, PassThroughKeepsOnlyBounds) {
    const std::vector<PointXYZ> cloud{{-2.0F, 0.0F, 0.0F},
                                      {0.0F, 0.0F, 0.0F},
                                      {2.0F, 0.0F, 0.0F},
                                      {0.5F, 0.5F, 1.5F}};
    FilterConfig config;
    config.x_min = -1.0F;
    config.x_max = 1.0F;
    config.y_min = -1.0F;
    config.y_max = 1.0F;
    config.z_min = -1.0F;
    config.z_max = 1.0F;
    config.enable_statistical_outlier_removal = false;

    const auto filtered = removeInvalidAndPassThrough(cloud, config);
    ASSERT_EQ(filtered.size(), 1U);
    EXPECT_NEAR(filtered[0].x, 0.0F, 1.0e-5F);
}

TEST(Filtering, StatisticalOutlierRemovalDropsIsolatedPoint) {
    const std::vector<PointXYZ> cloud{{0.00F, 0.00F, 0.0F},
                                      {0.05F, 0.02F, 0.0F},
                                      {0.10F, 0.00F, 0.0F},
                                      {0.12F, 0.04F, 0.0F},
                                      {20.0F, 20.0F, 0.0F}};

    const auto filtered = statisticalOutlierRemoval(cloud, 2, 0.5F, 0.25F);
    ASSERT_EQ(filtered.size(), 4U);
    for (const PointXYZ& point : filtered) {
        EXPECT_LT(point.x, 1.0F);
    }
}
