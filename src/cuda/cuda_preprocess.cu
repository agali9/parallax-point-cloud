#include "cuda_internal.hpp"

#include <chrono>
#include <stdexcept>
#include <string>
#include <vector>

#include <cuda_runtime.h>
#include <thrust/copy.h>
#include <thrust/device_vector.h>
#include <thrust/execution_policy.h>
#include <thrust/reduce.h>
#include <thrust/sort.h>
#include <thrust/transform.h>

#include "cuda_common.cuh"

namespace pointcloud_pipeline::cuda_detail {
namespace {

using Clock = std::chrono::steady_clock;

double elapsedMs(const Clock::time_point start, const Clock::time_point end) {
    return std::chrono::duration<double, std::milli>(end - start).count();
}

struct FilterBounds {
    float x_min;
    float x_max;
    float y_min;
    float y_max;
    float z_min;
    float z_max;
};

struct PassThroughPredicate {
    FilterBounds bounds;

    __host__ __device__ bool operator()(const PointXYZ& point) const {
        return isFinitePoint(point) && isInsideBounds(point, bounds.x_min, bounds.x_max,
                                                      bounds.y_min, bounds.y_max,
                                                      bounds.z_min, bounds.z_max);
    }
};

struct PointToSum {
    __host__ __device__ PointSum operator()(const PointXYZ& point) const {
        return PointSum{static_cast<double>(point.x),
                        static_cast<double>(point.y),
                        static_cast<double>(point.z),
                        1};
    }
};

struct SumToCentroid {
    __host__ __device__ PointXYZ operator()(const PointSum& sum) const {
        const double count = static_cast<double>(sum.count);
        return PointXYZ{static_cast<float>(sum.sum_x / count),
                        static_cast<float>(sum.sum_y / count),
                        static_cast<float>(sum.sum_z / count)};
    }
};

struct VoxelKeyEqual {
    __host__ __device__ bool operator()(const VoxelKey& lhs, const VoxelKey& rhs) const {
        return lhs == rhs;
    }
};

struct ComputeVoxelKey {
    float voxel_size;

    __host__ __device__ VoxelKey operator()(const PointXYZ& point) const {
        return voxelIndex(point, voxel_size);
    }
};

void checkCuda(cudaError_t status, const char* what) {
    if (status != cudaSuccess) {
        throw std::runtime_error(std::string(what) + ": " + cudaGetErrorString(status));
    }
}

std::vector<pointcloud_pipeline::PointXYZ> downloadHostPoints(
    const thrust::device_vector<PointXYZ>& device_points) {
    std::vector<pointcloud_pipeline::PointXYZ> host_points(device_points.size());
    if (!host_points.empty()) {
        checkCuda(cudaMemcpy(host_points.data(), thrust::raw_pointer_cast(device_points.data()),
                             host_points.size() * sizeof(PointXYZ), cudaMemcpyDeviceToHost),
                  "cudaMemcpy points");
    }
    return host_points;
}

}  // namespace

GpuPreprocessResult runGpuPreprocess(const pointcloud_pipeline::PointXYZ* host_points,
                                     std::size_t point_count,
                                     const pointcloud_pipeline::FilterConfig& filter,
                                     float voxel_size,
                                     bool enable_downsampling,
                                     bool filter_on_cpu) {
    GpuPreprocessResult result;
    if (point_count == 0U) {
        return result;
    }

    const auto* device_input =
        reinterpret_cast<const PointXYZ*>(host_points);

    const auto h2d_start = Clock::now();
    thrust::device_vector<PointXYZ> device_points(point_count);
    checkCuda(cudaMemcpy(thrust::raw_pointer_cast(device_points.data()), device_input,
                         point_count * sizeof(PointXYZ), cudaMemcpyHostToDevice),
              "cudaMemcpy input points");
    const auto h2d_end = Clock::now();
    result.timings.h2d_ms = elapsedMs(h2d_start, h2d_end);

    thrust::device_vector<PointXYZ> filtered_points;
    const auto filter_start = Clock::now();
    if (filter_on_cpu) {
        filtered_points.swap(device_points);
        result.filtered_cloud.assign(host_points, host_points + point_count);
    } else {
        const PassThroughPredicate predicate{filter.x_min, filter.x_max, filter.y_min,
                                             filter.y_max, filter.z_min, filter.z_max};
        filtered_points.resize(point_count);
        const auto out_end = thrust::copy_if(thrust::device, device_points.begin(),
                                             device_points.end(), filtered_points.begin(),
                                             predicate);
        filtered_points.resize(static_cast<std::size_t>(out_end - filtered_points.begin()));
    }
    const auto filter_end = Clock::now();
    result.timings.filter_ms = elapsedMs(filter_start, filter_end);

    if (!filter_on_cpu) {
        const auto d2h_filtered_start = Clock::now();
        result.filtered_cloud = downloadHostPoints(filtered_points);
        result.timings.d2h_ms += elapsedMs(d2h_filtered_start, Clock::now());
    }

    if (!enable_downsampling) {
        result.downsampled_cloud = result.filtered_cloud;
        return result;
    }

    if (filtered_points.empty()) {
        return result;
    }

    const auto voxel_start = Clock::now();
    thrust::device_vector<VoxelKey> voxel_keys(filtered_points.size());
    thrust::transform(thrust::device, filtered_points.begin(), filtered_points.end(),
                      voxel_keys.begin(), ComputeVoxelKey{voxel_size});
    thrust::sort_by_key(thrust::device, voxel_keys.begin(), voxel_keys.end(),
                        filtered_points.begin());

    thrust::device_vector<PointSum> point_sums(filtered_points.size());
    thrust::transform(thrust::device, filtered_points.begin(), filtered_points.end(),
                      point_sums.begin(), PointToSum{});

    thrust::device_vector<VoxelKey> reduced_keys(filtered_points.size());
    thrust::device_vector<PointSum> reduced_sums(filtered_points.size());
    const auto reduce_ends = thrust::reduce_by_key(
        thrust::device, voxel_keys.begin(), voxel_keys.end(), point_sums.begin(),
        reduced_keys.begin(), reduced_sums.begin(), VoxelKeyEqual{}, thrust::plus<PointSum>{});

    const std::size_t voxel_count =
        static_cast<std::size_t>(reduce_ends.first - reduced_keys.begin());
    reduced_keys.resize(voxel_count);
    reduced_sums.resize(voxel_count);

    thrust::device_vector<PointXYZ> centroids(voxel_count);
    thrust::transform(thrust::device, reduced_sums.begin(), reduced_sums.end(),
                      centroids.begin(), SumToCentroid{});

    const auto voxel_end = Clock::now();
    result.timings.voxel_ms = elapsedMs(voxel_start, voxel_end);

    const auto d2h_start = Clock::now();
    result.downsampled_cloud = downloadHostPoints(centroids);
    result.timings.d2h_ms += elapsedMs(d2h_start, Clock::now());

    return result;
}

}  // namespace pointcloud_pipeline::cuda_detail
