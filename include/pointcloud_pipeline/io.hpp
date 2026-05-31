#pragma once

#include <filesystem>
#include <vector>

#include "pointcloud_pipeline/types.hpp"

namespace pointcloud_pipeline {

// Load a KITTI Velodyne binary scan: packed float32 records of (x, y, z, intensity).
// Intensity is discarded. File size must be a multiple of 16 bytes.
[[nodiscard]] std::vector<PointXYZ> loadKittiBin(const std::filesystem::path& path);

}  // namespace pointcloud_pipeline
