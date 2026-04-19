#pragma once

#include <cstddef>
#include <vector>

namespace pointcloud_pipeline {

struct PointXYZ {
    float x = 0.0F;
    float y = 0.0F;
    float z = 0.0F;
};

struct BoundingBox {
    PointXYZ min;
    PointXYZ max;
};

struct Cluster {
    int id = 0;
    std::vector<std::size_t> indices;
    PointXYZ centroid;
    BoundingBox bbox;
};

struct StageTimings {
    double filter_ms = 0.0;
    double downsample_ms = 0.0;
    double segmentation_ms = 0.0;
    double total_ms = 0.0;
};

}  // namespace pointcloud_pipeline