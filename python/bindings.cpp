#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pointcloud_pipeline/filtering.hpp"
#include "pointcloud_pipeline/pipeline.hpp"
#include "pointcloud_pipeline/segmentation.hpp"
#include "pointcloud_pipeline/voxel_grid.hpp"

namespace py = pybind11;
namespace pcp = pointcloud_pipeline;

namespace {

struct PointArrayView {
    std::span<const pcp::PointXYZ> points;
    std::uintptr_t address;
};

PointArrayView pointViewFromArray(const py::array_t<float, py::array::c_style>& array) {
    const py::buffer_info info = array.request();
    if (info.ndim != 2 || info.shape[1] != 3) {
        throw std::invalid_argument("point cloud must have shape (N, 3)");
    }
    if (info.strides[1] != static_cast<py::ssize_t>(sizeof(float))) {
        throw std::invalid_argument("last dimension must be tightly packed float32 xyz values");
    }

    const auto* points = static_cast<const pcp::PointXYZ*>(info.ptr);
    const auto count = static_cast<std::size_t>(info.shape[0]);
    return PointArrayView{std::span<const pcp::PointXYZ>(points, count),
                          reinterpret_cast<std::uintptr_t>(info.ptr)};
}

py::array_t<float> vectorToNumpy(std::vector<pcp::PointXYZ>&& points) {
    auto* owned_points = new std::vector<pcp::PointXYZ>(std::move(points));
    py::capsule owner(owned_points, [](void* ptr) {
        delete static_cast<std::vector<pcp::PointXYZ>*>(ptr);
    });

    return py::array_t<float>(
        {static_cast<py::ssize_t>(owned_points->size()), py::ssize_t{3}},
        {static_cast<py::ssize_t>(sizeof(pcp::PointXYZ)), static_cast<py::ssize_t>(sizeof(float))},
        reinterpret_cast<float*>(owned_points->data()),
        owner);
}

py::dict pointToDict(const pcp::PointXYZ& point) {
    py::dict output;
    output["x"] = point.x;
    output["y"] = point.y;
    output["z"] = point.z;
    return output;
}

py::dict bboxToDict(const pcp::BoundingBox& bbox) {
    py::dict output;
    output["min"] = pointToDict(bbox.min);
    output["max"] = pointToDict(bbox.max);
    return output;
}

py::list clustersToPython(const std::vector<pcp::Cluster>& clusters) {
    py::list output;
    for (const pcp::Cluster& cluster : clusters) {
        py::dict item;
        item["id"] = cluster.id;
        item["indices"] = cluster.indices;
        item["centroid"] = pointToDict(cluster.centroid);
        item["bbox"] = bboxToDict(cluster.bbox);
        output.append(item);
    }
    return output;
}

pcp::FilterConfig filterConfigFromKwargs(const py::kwargs& kwargs) {
    pcp::FilterConfig config;
    if (kwargs.contains("x_min")) config.x_min = kwargs["x_min"].cast<float>();
    if (kwargs.contains("x_max")) config.x_max = kwargs["x_max"].cast<float>();
    if (kwargs.contains("y_min")) config.y_min = kwargs["y_min"].cast<float>();
    if (kwargs.contains("y_max")) config.y_max = kwargs["y_max"].cast<float>();
    if (kwargs.contains("z_min")) config.z_min = kwargs["z_min"].cast<float>();
    if (kwargs.contains("z_max")) config.z_max = kwargs["z_max"].cast<float>();
    if (kwargs.contains("enable_statistical_outlier_removal")) {
        config.enable_statistical_outlier_removal =
            kwargs["enable_statistical_outlier_removal"].cast<bool>();
    }
    if (kwargs.contains("neighbor_count")) config.neighbor_count = kwargs["neighbor_count"].cast<int>();
    if (kwargs.contains("std_dev_multiplier")) {
        config.std_dev_multiplier = kwargs["std_dev_multiplier"].cast<float>();
    }
    if (kwargs.contains("outlier_grid_cell_size")) {
        config.outlier_grid_cell_size = kwargs["outlier_grid_cell_size"].cast<float>();
    }
    return config;
}

pcp::SegmentationConfig segmentationConfigFromKwargs(const py::kwargs& kwargs) {
    pcp::SegmentationConfig config;
    if (kwargs.contains("cluster_tolerance")) {
        config.cluster_tolerance = kwargs["cluster_tolerance"].cast<float>();
    }
    if (kwargs.contains("min_cluster_size")) {
        config.min_cluster_size = kwargs["min_cluster_size"].cast<std::size_t>();
    }
    if (kwargs.contains("max_cluster_size")) {
        config.max_cluster_size = kwargs["max_cluster_size"].cast<std::size_t>();
    }
    return config;
}

pcp::PipelineConfig pipelineConfigFromKwargs(const py::kwargs& kwargs) {
    pcp::PipelineConfig config;
    config.filter = filterConfigFromKwargs(kwargs);
    config.segmentation = segmentationConfigFromKwargs(kwargs);
    if (kwargs.contains("voxel_size")) config.voxel.voxel_size = kwargs["voxel_size"].cast<float>();
    if (kwargs.contains("enable_downsampling")) {
        config.enable_downsampling = kwargs["enable_downsampling"].cast<bool>();
    }
    if (kwargs.contains("use_cuda")) {
        config.backend = kwargs["use_cuda"].cast<bool>() ? pcp::ExecutionBackend::CUDA
                                                         : pcp::ExecutionBackend::CPU;
    }
    return config;
}

}  // namespace

PYBIND11_MODULE(pointcloud_pipeline_py, module) {
    module.doc() = "Real-time LiDAR point cloud processing pipeline bindings";

    module.def("is_cuda_available", &pcp::isCudaAvailable,
               "Return True when the library was built with CUDA and a GPU is present.");
    module.def("cuda_device_name",
               []() -> py::object {
                   const char* name = pcp::cudaDeviceName();
                   if (name == nullptr) {
                       return py::none();
                   }
                   return py::str(name);
               },
               "Return the active CUDA device name, or None when unavailable.");

    module.def("numpy_data_address",
               [](const py::array_t<float, py::array::c_style>& cloud) {
                   return pointViewFromArray(cloud).address;
               },
               py::arg("cloud"),
               "Return the raw address of the NumPy buffer as seen by C++.");

    module.def("filter_cloud",
               [](const py::array_t<float, py::array::c_style>& cloud, py::kwargs kwargs) {
                   const PointArrayView view = pointViewFromArray(cloud);
                   return vectorToNumpy(pcp::filterCloud(view.points, filterConfigFromKwargs(kwargs)));
               },
               py::arg("cloud"));

    module.def("downsample_cloud",
               [](const py::array_t<float, py::array::c_style>& cloud, float voxel_size) {
                   const PointArrayView view = pointViewFromArray(cloud);
                   return vectorToNumpy(pcp::voxelGridDownsample(view.points, voxel_size));
               },
               py::arg("cloud"),
               py::arg("voxel_size") = 0.25F);

    module.def("segment_cloud",
               [](const py::array_t<float, py::array::c_style>& cloud, py::kwargs kwargs) {
                   const PointArrayView view = pointViewFromArray(cloud);
                   return clustersToPython(
                       pcp::euclideanCluster(view.points, segmentationConfigFromKwargs(kwargs)));
               },
               py::arg("cloud"));

    module.def("run_pipeline",
               [](const py::array_t<float, py::array::c_style>& cloud, py::kwargs kwargs) {
                   const PointArrayView view = pointViewFromArray(cloud);
                   const pcp::PointCloudPipeline pipeline(pipelineConfigFromKwargs(kwargs));
                   pcp::PipelineResult result = pipeline.process(view.points);

                   py::dict timings;
                   timings["filter_ms"] = result.timings.filter_ms;
                   timings["downsample_ms"] = result.timings.downsample_ms;
                   timings["segmentation_ms"] = result.timings.segmentation_ms;
                   timings["total_ms"] = result.timings.total_ms;
                   timings["h2d_ms"] = result.timings.h2d_ms;
                   timings["d2h_ms"] = result.timings.d2h_ms;
                   timings["used_cuda"] = result.timings.used_cuda;

                   py::dict output;
                   output["input_address"] = view.address;
                   output["filtered_cloud"] = vectorToNumpy(std::move(result.filtered_cloud));
                   output["downsampled_cloud"] = vectorToNumpy(std::move(result.downsampled_cloud));
                   output["clusters"] = clustersToPython(result.clusters);
                   output["timings"] = timings;
                   return output;
               },
               py::arg("cloud"));
}
