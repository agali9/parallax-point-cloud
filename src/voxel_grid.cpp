#include "pointcloud_pipeline/voxel_grid.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace pointcloud_pipeline {
namespace {

struct VoxelAccumulator {
    std::size_t count = 0;
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
};

std::uint64_t splitMix64(std::uint64_t value) noexcept {
    value += 0x9e3779b97f4a7c15ULL;
    value = (value ^ (value >> 30U)) * 0xbf58476d1ce4e5b9ULL;
    value = (value ^ (value >> 27U)) * 0x94d049bb133111ebULL;
    return value ^ (value >> 31U);
}

}  // namespace

bool VoxelKey::operator<(const VoxelKey& other) const {
    if (x != other.x) {
        return x < other.x;
    }
    if (y != other.y) {
        return y < other.y;
    }
    return z < other.z;
}

std::size_t VoxelKeyHash::operator()(const VoxelKey& key) const noexcept {
    const auto hx = splitMix64(static_cast<std::uint64_t>(key.x));
    const auto hy = splitMix64(static_cast<std::uint64_t>(key.y));
    const auto hz = splitMix64(static_cast<std::uint64_t>(key.z));
    return static_cast<std::size_t>(hx ^ (hy << 1U) ^ (hz << 2U));
}

VoxelKey voxelIndex(const PointXYZ& point, float voxel_size) {
    if (voxel_size <= 0.0F) {
        throw std::invalid_argument("voxel_size must be positive");
    }
    return VoxelKey{static_cast<std::int64_t>(std::floor(point.x / voxel_size)),
                    static_cast<std::int64_t>(std::floor(point.y / voxel_size)),
                    static_cast<std::int64_t>(std::floor(point.z / voxel_size))};
}

std::vector<PointXYZ> voxelGridDownsample(std::span<const PointXYZ> cloud,
                                          float voxel_size) {
    if (voxel_size <= 0.0F) {
        throw std::invalid_argument("voxel_size must be positive");
    }

    std::unordered_map<VoxelKey, VoxelAccumulator, VoxelKeyHash> voxels;
    voxels.reserve(cloud.size());

    for (const PointXYZ& point : cloud) {
        VoxelAccumulator& accumulator = voxels[voxelIndex(point, voxel_size)];
        ++accumulator.count;
        accumulator.sum_x += point.x;
        accumulator.sum_y += point.y;
        accumulator.sum_z += point.z;
    }

    std::vector<std::pair<VoxelKey, VoxelAccumulator>> sorted_voxels;
    sorted_voxels.reserve(voxels.size());
    for (const auto& entry : voxels) {
        sorted_voxels.push_back(entry);
    }
    std::sort(sorted_voxels.begin(), sorted_voxels.end(),
              [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });

    std::vector<PointXYZ> output;
    output.reserve(sorted_voxels.size());
    for (const auto& [key, accumulator] : sorted_voxels) {
        (void)key;
        const auto count = static_cast<double>(accumulator.count);
        output.push_back(PointXYZ{static_cast<float>(accumulator.sum_x / count),
                                  static_cast<float>(accumulator.sum_y / count),
                                  static_cast<float>(accumulator.sum_z / count)});
    }

    return output;
}

}  // namespace pointcloud_pipeline
