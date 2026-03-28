#include "minigtest.hpp"
#include "pointcloud_pipeline/voxel_grid.hpp"
#include "test_common.hpp"

#include <vector>

TEST(VoxelGrid, CombinesPointsInSameCell) {
    std::vector<pointcloud_pipeline::PointXYZ> cloud{makePoint(0.0F, 0, 0), makePoint(0.1F, 0, 0)};
    const auto downsampled = pointcloud_pipeline::voxelGridDownsample(cloud, 0.25F);
    EXPECT_EQ(downsampled.size(), 1U);
    EXPECT_NEAR(downsampled[0].x, 0.05F, 1.0e-5F);
}