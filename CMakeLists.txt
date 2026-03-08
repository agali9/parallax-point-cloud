cmake_minimum_required(VERSION 3.20)
project(pointcloud_pipeline LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(pointcloud_pipeline SHARED
    src/open3d_bridge.cpp)

target_include_directories(pointcloud_pipeline PUBLIC include)

find_package(Open3D QUIET)
if(Open3D_FOUND)
    target_compile_definitions(pointcloud_pipeline PUBLIC POINTCLOUD_PIPELINE_WITH_OPEN3D)
    target_link_libraries(pointcloud_pipeline PUBLIC Open3D::Open3D)
endif()

add_executable(pointcloud_pipeline_example examples/cpp_example.cpp)
target_link_libraries(pointcloud_pipeline_example PRIVATE pointcloud_pipeline)