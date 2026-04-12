#include <chrono>
#include <iostream>
#include <random>
#include <vector>

#include "pointcloud_pipeline/pipeline.hpp"

namespace {

std::vector<pointcloud_pipeline::PointXYZ> makeCloud(std::size_t count) {
    std::mt19937 rng(42);
    std::uniform_real_distribution<float> dist(-20.0F, 20.0F);
    std::vector<pointcloud_pipeline::PointXYZ> cloud;
    for (std::size_t i = 0; i < count; ++i) {
        cloud.push_back({dist(rng), dist(rng), dist(rng)});
    }
    return cloud;
}

}  // namespace

int main() {
    const auto cloud = makeCloud(100000);
    pointcloud_pipeline::PipelineConfig config;
    config.segmentation.min_cluster_size = 3;
    pointcloud_pipeline::PointCloudPipeline pipeline(config);

    const auto start = std::chrono::steady_clock::now();
    const auto result = pipeline.process(cloud);
    const auto end = std::chrono::steady_clock::now();

    const auto ms = std::chrono::duration<double, std::milli>(end - start).count();
    std::cout << "points=" << cloud.size()
              << " downsampled=" << result.downsampled_cloud.size()
              << " clusters=" << result.clusters.size()
              << " ms=" << ms << '\n';
}