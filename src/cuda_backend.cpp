#include "pointcloud_pipeline/cuda_backend.hpp"

#include <stdexcept>
#include <string>

#ifdef POINTCLOUD_PIPELINE_WITH_CUDA
#include <cuda_runtime.h>

#include "cuda/cuda_internal.hpp"
#endif

namespace pointcloud_pipeline {

bool isCudaAvailable() noexcept {
#ifdef POINTCLOUD_PIPELINE_WITH_CUDA
    int device_count = 0;
    if (cudaGetDeviceCount(&device_count) != cudaSuccess) {
        return false;
    }
    return device_count > 0;
#else
    return false;
#endif
}

const char* cudaDeviceName() noexcept {
#ifdef POINTCLOUD_PIPELINE_WITH_CUDA
    if (!isCudaAvailable()) {
        return nullptr;
    }

    static thread_local std::string device_name;
    cudaDeviceProp properties{};
    if (cudaGetDeviceProperties(&properties, 0) != cudaSuccess) {
        return nullptr;
    }
    device_name = properties.name;
    return device_name.c_str();
#else
    return nullptr;
#endif
}

bool shouldUseCuda(const PipelineConfig& config) noexcept {
#ifdef POINTCLOUD_PIPELINE_WITH_CUDA
    if (config.backend == ExecutionBackend::CPU) {
        return false;
    }
    if (config.backend == ExecutionBackend::CUDA) {
        return isCudaAvailable();
    }
    return isCudaAvailable();
#else
    (void)config;
    return false;
#endif
}

GpuPreprocessResult gpuPreprocess(std::span<const PointXYZ> cloud,
                                  const FilterConfig& filter,
                                  float voxel_size,
                                  bool enable_downsampling,
                                  bool filter_on_cpu) {
#ifdef POINTCLOUD_PIPELINE_WITH_CUDA
    if (!isCudaAvailable()) {
        throw std::runtime_error("CUDA was requested but no compatible GPU is available");
    }
    return cuda_detail::runGpuPreprocess(cloud.data(), cloud.size(), filter, voxel_size,
                                         enable_downsampling, filter_on_cpu);
#else
    (void)cloud;
    (void)filter;
    (void)voxel_size;
    (void)enable_downsampling;
    (void)filter_on_cpu;
    throw std::runtime_error("CUDA support was not compiled into pointcloud_pipeline");
#endif
}

}  // namespace pointcloud_pipeline
