#pragma once

#include <cmath>
#include <cstdint>

namespace pointcloud_pipeline::cuda_detail {

struct PointXYZ {
    float x;
    float y;
    float z;
};

struct VoxelKey {
    std::int64_t x;
    std::int64_t y;
    std::int64_t z;

    __host__ __device__ bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }

    __host__ __device__ bool operator<(const VoxelKey& other) const {
        if (x != other.x) {
            return x < other.x;
        }
        if (y != other.y) {
            return y < other.y;
        }
        return z < other.z;
    }
};

struct PointSum {
    double sum_x = 0.0;
    double sum_y = 0.0;
    double sum_z = 0.0;
    int count = 0;
};

__host__ __device__ inline PointSum operator+(const PointSum& lhs, const PointSum& rhs) {
    return PointSum{lhs.sum_x + rhs.sum_x,
                    lhs.sum_y + rhs.sum_y,
                    lhs.sum_z + rhs.sum_z,
                    lhs.count + rhs.count};
}

__host__ __device__ inline bool isFinitePoint(const PointXYZ& point) {
    return isfinite(point.x) && isfinite(point.y) && isfinite(point.z);
}

__host__ __device__ inline bool isInsideBounds(const PointXYZ& point,
                                               float x_min,
                                               float x_max,
                                               float y_min,
                                               float y_max,
                                               float z_min,
                                               float z_max) {
    return point.x >= x_min && point.x <= x_max && point.y >= y_min && point.y <= y_max &&
           point.z >= z_min && point.z <= z_max;
}

__host__ __device__ inline VoxelKey voxelIndex(const PointXYZ& point, float voxel_size) {
    return VoxelKey{static_cast<std::int64_t>(floorf(point.x / voxel_size)),
                    static_cast<std::int64_t>(floorf(point.y / voxel_size)),
                    static_cast<std::int64_t>(floorf(point.z / voxel_size))};
}

}  // namespace pointcloud_pipeline::cuda_detail
