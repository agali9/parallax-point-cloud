#include "pointcloud_pipeline/open3d_bridge.hpp"

#ifdef POINTCLOUD_PIPELINE_WITH_OPEN3D

namespace pointcloud_pipeline {

std::vector<PointXYZ> fromOpen3DPointCloud(const open3d::geometry::PointCloud& cloud) {
    std::vector<PointXYZ> points;
    points.reserve(cloud.points_.size());
    for (const Eigen::Vector3d& point : cloud.points_) {
        points.push_back(PointXYZ{static_cast<float>(point.x()),
                                  static_cast<float>(point.y()),
                                  static_cast<float>(point.z())});
    }
    return points;
}

open3d::geometry::PointCloud toOpen3DPointCloud(const std::vector<PointXYZ>& cloud) {
    open3d::geometry::PointCloud output;
    output.points_.reserve(cloud.size());
    for (const PointXYZ& point : cloud) {
        output.points_.emplace_back(point.x, point.y, point.z);
    }
    return output;
}

}  // namespace pointcloud_pipeline

#endif
