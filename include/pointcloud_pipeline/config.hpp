#pragma once

#include <cstddef>
#include <limits>

namespace pointcloud_pipeline {

struct FilterConfig {
    float x_min = -std::numeric_limits<float>::infinity();
    float x_max = std::numeric_limits<float>::infinity();
    float y_min = -std::numeric_limits<float>::infinity();
    float y_max = std::numeric_limits<float>::infinity();
    float z_min = -std::numeric_limits<float>::infinity();
    float z_max = std::numeric_limits<float>::infinity();
};

struct VoxelGridConfig {
    float voxel_size = 0.25F;
};

struct SegmentationConfig {
    float cluster_tolerance = 0.65F;
    std::size_t min_cluster_size = 8;
    std::size_t max_cluster_size = 250000;
};

struct PipelineConfig {
    FilterConfig filter;
    VoxelGridConfig voxel;
    SegmentationConfig segmentation;
    bool enable_downsampling = true;
};

}  // namespace pointcloud_pipeline