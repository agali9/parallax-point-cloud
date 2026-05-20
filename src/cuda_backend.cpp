#include "pointcloud_pipeline/cuda_backend.hpp"

namespace pointcloud_pipeline {

bool isCudaAvailable() noexcept { return false; }

const char* cudaDeviceName() noexcept { return nullptr; }

}  // namespace pointcloud_pipeline
