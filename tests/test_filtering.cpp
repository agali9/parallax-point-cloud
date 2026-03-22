#include "minigtest.hpp"
#include "pointcloud_pipeline/filtering.hpp"
#include "test_common.hpp"

#include <limits>
#include <vector>

TEST(Filtering, RemovesInvalidPoints) {
    const float nan = std::numeric_limits<float>::quiet_NaN();
    std::vector<pointcloud_pipeline::PointXYZ> cloud{makePoint(0, 0, 0), makePoint(nan, 0, 0)};
    const auto filtered = pointcloud_pipeline::filterCloud(cloud, pointcloud_pipeline::FilterConfig{});
    EXPECT_EQ(filtered.size(), 1U);
}

TEST(Filtering, AppliesBounds) {
    std::vector<pointcloud_pipeline::PointXYZ> cloud{makePoint(0, 0, 0), makePoint(10, 0, 0)};
    pointcloud_pipeline::FilterConfig config;
    config.x_max = 1.0F;
    const auto filtered = pointcloud_pipeline::filterCloud(cloud, config);
    EXPECT_EQ(filtered.size(), 1U);
}