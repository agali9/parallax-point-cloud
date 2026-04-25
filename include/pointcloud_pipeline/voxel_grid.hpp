#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "pointcloud_pipeline/types.hpp"

namespace pointcloud_pipeline {

struct VoxelKey {
    std::int64_t x;
    std::int64_t y;
    std::int64_t z;

    bool operator==(const VoxelKey& other) const = default;
    bool operator<(const VoxelKey& other) const;
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const noexcept;
};

VoxelKey voxelIndex(const PointXYZ& point, float voxel_size);

std::vector<PointXYZ> voxelGridDownsample(std::span<const PointXYZ> cloud,
                                          float voxel_size);

}  // namespace pointcloud_pipeline
