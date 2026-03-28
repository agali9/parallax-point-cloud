#include "pointcloud_pipeline/voxel_grid.hpp"

#include <cmath>
#include <sstream>
#include <string>
#include <unordered_map>

namespace pointcloud_pipeline {
namespace {

struct Accumulator {
    float sum_x = 0.0F;
    float sum_y = 0.0F;
    float sum_z = 0.0F;
    int count = 0;
};

std::string makeKey(const PointXYZ& point, float voxel_size) {
    std::ostringstream out;
    out << static_cast<int>(std::floor(point.x / voxel_size)) << ':'
        << static_cast<int>(std::floor(point.y / voxel_size)) << ':'
        << static_cast<int>(std::floor(point.z / voxel_size));
    return out.str();
}

}  // namespace

std::vector<PointXYZ> voxelGridDownsample(const std::vector<PointXYZ>& cloud,
                                          float voxel_size) {
    if (voxel_size <= 0.0F) {
        return cloud;
    }

    std::unordered_map<std::string, Accumulator> voxels;
    for (const PointXYZ& point : cloud) {
        auto& accum = voxels[makeKey(point, voxel_size)];
        accum.sum_x += point.x;
        accum.sum_y += point.y;
        accum.sum_z += point.z;
        ++accum.count;
    }

    std::vector<PointXYZ> output;
    for (const auto& [key, accum] : voxels) {
        (void)key;
        output.push_back(PointXYZ{accum.sum_x / accum.count,
                                  accum.sum_y / accum.count,
                                  accum.sum_z / accum.count});
    }
    return output;
}

}  // namespace pointcloud_pipeline