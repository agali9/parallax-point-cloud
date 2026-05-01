#include "test_common.hpp"

#include <vector>

#include "pointcloud_pipeline/voxel_grid.hpp"

using pointcloud_pipeline::PointXYZ;
using pointcloud_pipeline::voxelGridDownsample;
using pointcloud_pipeline::voxelIndex;

TEST(VoxelGrid, ReducesPointCount) {
    const std::vector<PointXYZ> cloud{{0.00F, 0.00F, 0.00F},
                                      {0.05F, 0.05F, 0.05F},
                                      {1.00F, 1.00F, 1.00F}};

    const auto downsampled = voxelGridDownsample(cloud, 0.25F);
    EXPECT_EQ(downsampled.size(), 2U);
}

TEST(VoxelGrid, ComputesVoxelCentroid) {
    const std::vector<PointXYZ> cloud{{0.00F, 0.00F, 0.00F},
                                      {0.10F, 0.20F, 0.30F}};

    const auto downsampled = voxelGridDownsample(cloud, 1.0F);
    ASSERT_EQ(downsampled.size(), 1U);
    EXPECT_NEAR(downsampled[0].x, 0.05F, 1.0e-5F);
    EXPECT_NEAR(downsampled[0].y, 0.10F, 1.0e-5F);
    EXPECT_NEAR(downsampled[0].z, 0.15F, 1.0e-5F);
}

TEST(VoxelGrid, HandlesNegativeVoxelIndexing) {
    const auto key = voxelIndex(PointXYZ{-0.01F, -1.01F, 2.0F}, 1.0F);
    EXPECT_EQ(key.x, -1);
    EXPECT_EQ(key.y, -2);
    EXPECT_EQ(key.z, 2);
}
