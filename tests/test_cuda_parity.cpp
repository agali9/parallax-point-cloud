#include "test_common.hpp"

#include <cmath>
#include <iostream>
#include <vector>

#include "pointcloud_pipeline/config.hpp"
#include "pointcloud_pipeline/cuda_backend.hpp"
#include "pointcloud_pipeline/filtering.hpp"
#include "pointcloud_pipeline/pipeline.hpp"
#include "pointcloud_pipeline/voxel_grid.hpp"

using pointcloud_pipeline::ExecutionBackend;
using pointcloud_pipeline::FilterConfig;
using pointcloud_pipeline::PipelineConfig;
using pointcloud_pipeline::PointCloudPipeline;
using pointcloud_pipeline::PointXYZ;
using pointcloud_pipeline::filterCloud;
using pointcloud_pipeline::gpuPreprocess;
using pointcloud_pipeline::isCudaAvailable;
using pointcloud_pipeline::voxelGridDownsample;

namespace {

bool cudaTestsEnabled() {
    if (!isCudaAvailable()) {
        return false;
    }
    return true;
}

void skipIfNoCuda(const char* test_name) {
    if (!cudaTestsEnabled()) {
        std::cout << "[  SKIPPED ] " << test_name << " (CUDA device unavailable)\n";
    }
}

bool nearPoint(const PointXYZ& lhs, const PointXYZ& rhs, float tolerance) {
    return std::abs(lhs.x - rhs.x) <= tolerance && std::abs(lhs.y - rhs.y) <= tolerance &&
           std::abs(lhs.z - rhs.z) <= tolerance;
}

bool cloudsMatch(const std::vector<PointXYZ>& lhs,
                 const std::vector<PointXYZ>& rhs,
                 float tolerance) {
    if (lhs.size() != rhs.size()) {
        return false;
    }
    for (std::size_t i = 0; i < lhs.size(); ++i) {
        if (!nearPoint(lhs[i], rhs[i], tolerance)) {
            return false;
        }
    }
    return true;
}

std::vector<PointXYZ> sampleCloud() {
    return std::vector<PointXYZ>{{0.00F, 0.00F, 0.00F},
                                 {0.05F, 0.05F, 0.05F},
                                 {0.10F, 0.20F, 0.30F},
                                 {1.00F, 1.00F, 1.00F},
                                 {1.05F, 1.05F, 1.05F},
                                 {20.0F, 20.0F, 20.0F}};
}

}  // namespace

TEST(CudaParity, FilterMatchesCpu) {
    skipIfNoCuda("CudaParity.FilterMatchesCpu");
    if (!cudaTestsEnabled()) {
        return;
    }

    const std::vector<PointXYZ> cloud = sampleCloud();
    FilterConfig config;
    config.enable_statistical_outlier_removal = false;
    config.x_min = -2.0F;
    config.x_max = 2.0F;
    config.y_min = -2.0F;
    config.y_max = 2.0F;
    config.z_min = -2.0F;
    config.z_max = 2.0F;

    const auto cpu_filtered = filterCloud(cloud, config);
    const auto gpu_filtered =
        gpuPreprocess(cloud, config, 0.25F, false, false).filtered_cloud;

    ASSERT_EQ(cpu_filtered.size(), gpu_filtered.size());
    EXPECT_TRUE(cloudsMatch(cpu_filtered, gpu_filtered, 1.0e-4F));
}

TEST(CudaParity, VoxelMatchesCpu) {
    skipIfNoCuda("CudaParity.VoxelMatchesCpu");
    if (!cudaTestsEnabled()) {
        return;
    }

    const std::vector<PointXYZ> cloud = sampleCloud();
    FilterConfig config;
    config.enable_statistical_outlier_removal = false;

    const auto cpu_filtered = filterCloud(cloud, config);
    const auto cpu_downsampled = voxelGridDownsample(cpu_filtered, 0.25F);
    const auto gpu_downsampled =
        gpuPreprocess(cloud, config, 0.25F, true, false).downsampled_cloud;

    ASSERT_EQ(cpu_downsampled.size(), gpu_downsampled.size());
    EXPECT_TRUE(cloudsMatch(cpu_downsampled, gpu_downsampled, 1.0e-4F));
}

TEST(CudaParity, FullPipelineClustersMatch) {
    skipIfNoCuda("CudaParity.FullPipelineClustersMatch");
    if (!cudaTestsEnabled()) {
        return;
    }

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

    PipelineConfig cpu_config = config;
    cpu_config.backend = ExecutionBackend::CPU;
    PipelineConfig cuda_config = config;
    cuda_config.backend = ExecutionBackend::CUDA;

    const PointCloudPipeline cpu_pipeline(cpu_config);
    const PointCloudPipeline cuda_pipeline(cuda_config);

    const auto cpu_result = cpu_pipeline.process(cloud);
    const auto cuda_result = cuda_pipeline.process(cloud);

    EXPECT_TRUE(cuda_result.timings.used_cuda);
    ASSERT_EQ(cpu_result.clusters.size(), cuda_result.clusters.size());
    for (std::size_t i = 0; i < cpu_result.clusters.size(); ++i) {
        EXPECT_EQ(cpu_result.clusters[i].indices.size(),
                  cuda_result.clusters[i].indices.size());
        EXPECT_NEAR(cpu_result.clusters[i].centroid.x, cuda_result.clusters[i].centroid.x,
                    1.0e-3F);
        EXPECT_NEAR(cpu_result.clusters[i].centroid.y, cuda_result.clusters[i].centroid.y,
                    1.0e-3F);
    }
}
