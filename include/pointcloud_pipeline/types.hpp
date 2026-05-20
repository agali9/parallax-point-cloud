#pragma once

#include <cstddef>
#include <vector>

namespace pointcloud_pipeline {

struct PointXYZ {
    float x;
    float y;
    float z;
};

struct BoundingBox {
    PointXYZ min;
    PointXYZ max;
};

struct Cluster {
    int id;
    std::vector<std::size_t> indices;
    PointXYZ centroid;
    BoundingBox bbox;
};

struct StageTimings {
    double filter_ms;
    double downsample_ms;
    double segmentation_ms;
    double total_ms;
    double h2d_ms = 0.0;
    double d2h_ms = 0.0;
    bool used_cuda = false;
};

struct PipelineResult {
    std::vector<PointXYZ> filtered_cloud;
    std::vector<PointXYZ> downsampled_cloud;
    std::vector<Cluster> clusters;
    StageTimings timings;
};

static_assert(sizeof(PointXYZ) == 3 * sizeof(float),
              "PointXYZ must stay tightly packed for NumPy interop");

}  // namespace pointcloud_pipeline
