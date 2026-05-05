#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "pointcloud_pipeline/pipeline.hpp"

namespace py = pybind11;

PYBIND11_MODULE(pointcloud_pipeline_py, m) {
    py::class_<pointcloud_pipeline::PointXYZ>(m, "PointXYZ")
        .def(py::init<>())
        .def_readwrite("x", &pointcloud_pipeline::PointXYZ::x)
        .def_readwrite("y", &pointcloud_pipeline::PointXYZ::y)
        .def_readwrite("z", &pointcloud_pipeline::PointXYZ::z);

    py::class_<pointcloud_pipeline::PipelineConfig>(m, "PipelineConfig")
        .def(py::init<>());

    py::class_<pointcloud_pipeline::PointCloudPipeline>(m, "PointCloudPipeline")
        .def(py::init<>())
        .def("process", [](const pointcloud_pipeline::PointCloudPipeline& pipeline,
                           const std::vector<pointcloud_pipeline::PointXYZ>& points) {
            return pipeline.process(points);
        });
}