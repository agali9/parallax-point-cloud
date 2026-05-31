#pragma once

#include <vector>

#include "pointcloud_pipeline/types.hpp"

#ifdef POINTCLOUD_PIPELINE_WITH_OPEN3D
#include <open3d/Open3D.h>
#endif

namespace pointcloud_pipeline {

#ifdef POINTCLOUD_PIPELINE_WITH_OPEN3D
std::vector<PointXYZ> fromOpen3DPointCloud(const open3d::geometry::PointCloud& cloud);
open3d::geometry::PointCloud toOpen3DPointCloud(const std::vector<PointXYZ>& cloud);
#endif

}  // namespace pointcloud_pipeline
