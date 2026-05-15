cmake_minimum_required(VERSION 3.20)
project(pointcloud_pipeline_ros)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pointcloud_pipeline REQUIRED CONFIG)

add_executable(pointcloud_pipeline_node src/pipeline_node.cpp)
target_include_directories(pointcloud_pipeline_node PRIVATE include)
target_link_libraries(pointcloud_pipeline_node pointcloud_pipeline::pointcloud_pipeline)
ament_target_dependencies(pointcloud_pipeline_node
    rclcpp
    sensor_msgs
    visualization_msgs
    geometry_msgs
    std_msgs)

if(MSVC)
    target_compile_options(pointcloud_pipeline_node PRIVATE /W4 /permissive-)
else()
    target_compile_options(pointcloud_pipeline_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

install(TARGETS pointcloud_pipeline_node
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME})

ament_package()
