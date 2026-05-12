cmake_minimum_required(VERSION 3.8)
project(pointcloud_pipeline_ros)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pointcloud_pipeline REQUIRED)

add_executable(pipeline_node src/pipeline_node.cpp)
target_link_libraries(pipeline_node pointcloud_pipeline::pointcloud_pipeline)
ament_target_dependencies(pipeline_node rclcpp)

install(TARGETS pipeline_node DESTINATION lib/${PROJECT_NAME})
ament_package()