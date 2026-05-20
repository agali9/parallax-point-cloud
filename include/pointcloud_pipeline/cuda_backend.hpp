#pragma once

#include "pointcloud_pipeline/config.hpp"
#include "pointcloud_pipeline/types.hpp"

namespace pointcloud_pipeline {

[[nodiscard]] bool isCudaAvailable() noexcept;
[[nodiscard]] const char* cudaDeviceName() noexcept;

}  // namespace pointcloud_pipeline
