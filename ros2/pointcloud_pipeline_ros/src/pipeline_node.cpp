#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "pointcloud_pipeline/pipeline.hpp"

class PipelineNode : public rclcpp::Node {
public:
    PipelineNode() : Node("pointcloud_pipeline_node") {
        RCLCPP_INFO(get_logger(), "point cloud pipeline node started");
    }

private:
    pointcloud_pipeline::PointCloudPipeline pipeline_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PipelineNode>());
    rclcpp::shutdown();
    return 0;
}