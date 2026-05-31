#pragma once

#include <cstddef>
#include <limits>

namespace pointcloud_pipeline {

enum class ExecutionBackend {
    Auto,
    CPU,
    CUDA,
};

struct FilterConfig {
    float x_min = -std::numeric_limits<float>::infinity();
    float x_max = std::numeric_limits<float>::infinity();
    float y_min = -std::numeric_limits<float>::infinity();
    float y_max = std::numeric_limits<float>::infinity();
    float z_min = -std::numeric_limits<float>::infinity();
    float z_max = std::numeric_limits<float>::infinity();

    bool enable_statistical_outlier_removal = true;
    int neighbor_count = 8;
    float std_dev_multiplier = 1.0F;

    // The classic statistical outlier filter is based on k nearest neighbors.
    // This cell size only controls how quickly we find local candidates.
    float outlier_grid_cell_size = 1.0F;
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
    ExecutionBackend backend = ExecutionBackend::Auto;
};

}  // namespace pointcloud_pipeline
