#include "pointcloud_pipeline/filtering.hpp"

#include <cmath>

namespace pointcloud_pipeline {
namespace {

bool isFinitePoint(const PointXYZ& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

bool isInsideBounds(const PointXYZ& point, const FilterConfig& config) {
    return point.x >= config.x_min && point.x <= config.x_max &&
           point.y >= config.y_min && point.y <= config.y_max &&
           point.z >= config.z_min && point.z <= config.z_max;
}

}  // namespace

std::vector<PointXYZ> removeInvalidAndPassThrough(const std::vector<PointXYZ>& cloud,
                                                  const FilterConfig& config) {
    std::vector<PointXYZ> filtered;
    for (const PointXYZ& point : cloud) {
        if (isFinitePoint(point) && isInsideBounds(point, config)) {
            filtered.push_back(point);
        }
    }
    return filtered;
}

std::vector<PointXYZ> filterCloud(const std::vector<PointXYZ>& cloud,
                                  const FilterConfig& config) {
    return removeInvalidAndPassThrough(cloud, config);
}

}  // namespace pointcloud_pipeline