#pragma once

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

struct PipelineConfig {
    FilterConfig filter;
    VoxelGridConfig voxel;
    bool enable_downsampling = true;
};

}  // namespace pointcloud_pipeline