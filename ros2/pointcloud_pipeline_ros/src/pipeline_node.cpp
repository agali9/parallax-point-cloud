#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "pointcloud_pipeline/pipeline.hpp"

namespace {

using pointcloud_pipeline::PointXYZ;

std::vector<PointXYZ> cloudFromRosMessage(const sensor_msgs::msg::PointCloud2& message) {
    std::vector<PointXYZ> points;
    points.reserve(static_cast<std::size_t>(message.width) *
                   static_cast<std::size_t>(message.height));

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(message, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(message, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(message, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        points.push_back(PointXYZ{*iter_x, *iter_y, *iter_z});
    }

    return points;
}

sensor_msgs::msg::PointCloud2 cloudToRosMessage(
    const std::vector<PointXYZ>& points,
    const std_msgs::msg::Header& header) {
    sensor_msgs::msg::PointCloud2 message;
    message.header = header;
    message.height = 1;
    message.width = static_cast<std::uint32_t>(points.size());
    message.is_dense = true;
    message.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(message);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(message, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(message, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(message, "z");

    for (const PointXYZ& point : points) {
        *iter_x = point.x;
        *iter_y = point.y;
        *iter_z = point.z;
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }

    return message;
}

std_msgs::msg::ColorRGBA clusterColor(int id) {
    const float hue = static_cast<float>((id * 47) % 360) / 60.0F;
    const int sector = static_cast<int>(std::floor(hue));
    const float fraction = hue - static_cast<float>(sector);
    const float q = 1.0F - fraction;

    float red = 0.0F;
    float green = 0.0F;
    float blue = 0.0F;
    switch (sector % 6) {
        case 0: red = 1.0F; green = fraction; blue = 0.0F; break;
        case 1: red = q; green = 1.0F; blue = 0.0F; break;
        case 2: red = 0.0F; green = 1.0F; blue = fraction; break;
        case 3: red = 0.0F; green = q; blue = 1.0F; break;
        case 4: red = fraction; green = 0.0F; blue = 1.0F; break;
        default: red = 1.0F; green = 0.0F; blue = q; break;
    }

    std_msgs::msg::ColorRGBA color;
    color.r = red;
    color.g = green;
    color.b = blue;
    color.a = 0.45F;
    return color;
}

visualization_msgs::msg::MarkerArray markersFromClusters(
    const std::vector<pointcloud_pipeline::Cluster>& clusters,
    const std_msgs::msg::Header& header) {
    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header = header;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    array.markers.push_back(clear_marker);

    for (const auto& cluster : clusters) {
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = "pointcloud_pipeline_clusters";
        marker.id = cluster.id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = 0.5 * (cluster.bbox.min.x + cluster.bbox.max.x);
        marker.pose.position.y = 0.5 * (cluster.bbox.min.y + cluster.bbox.max.y);
        marker.pose.position.z = 0.5 * (cluster.bbox.min.z + cluster.bbox.max.z);
        marker.pose.orientation.w = 1.0;

        marker.scale.x = std::max(0.05F, cluster.bbox.max.x - cluster.bbox.min.x);
        marker.scale.y = std::max(0.05F, cluster.bbox.max.y - cluster.bbox.min.y);
        marker.scale.z = std::max(0.05F, cluster.bbox.max.z - cluster.bbox.min.z);
        marker.color = clusterColor(cluster.id);
        marker.lifetime = rclcpp::Duration::from_seconds(0.25);
        array.markers.push_back(marker);
    }

    return array;
}

}  // namespace

class PointCloudPipelineNode final : public rclcpp::Node {
public:
    PointCloudPipelineNode()
        : rclcpp::Node("pointcloud_pipeline_node"),
          pipeline_(loadPipelineConfig()) {
        const std::string input_topic =
            declare_parameter<std::string>("input_topic", "/points_raw");
        processed_pub_ =
            create_publisher<sensor_msgs::msg::PointCloud2>("processed_cloud", 10);
        markers_pub_ =
            create_publisher<visualization_msgs::msg::MarkerArray>("cluster_markers", 10);

        subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
                handleCloud(std::move(message));
            });

        RCLCPP_INFO(get_logger(), "PointCloudPipeline ROS node listening on %s",
                    input_topic.c_str());
    }

private:
    pointcloud_pipeline::PipelineConfig loadPipelineConfig() {
        pointcloud_pipeline::PipelineConfig config;
        config.filter.x_min = declare_parameter<double>("x_min", -80.0);
        config.filter.x_max = declare_parameter<double>("x_max", 80.0);
        config.filter.y_min = declare_parameter<double>("y_min", -80.0);
        config.filter.y_max = declare_parameter<double>("y_max", 80.0);
        config.filter.z_min = declare_parameter<double>("z_min", -3.0);
        config.filter.z_max = declare_parameter<double>("z_max", 5.0);
        config.filter.enable_statistical_outlier_removal =
            declare_parameter<bool>("enable_statistical_outlier_removal", false);
        config.filter.neighbor_count = declare_parameter<int>("neighbor_count", 8);
        config.filter.std_dev_multiplier =
            declare_parameter<double>("std_dev_multiplier", 1.0);
        config.filter.outlier_grid_cell_size =
            declare_parameter<double>("outlier_grid_cell_size", 0.5);
        config.enable_downsampling = declare_parameter<bool>("enable_downsampling", true);
        config.voxel.voxel_size = declare_parameter<double>("voxel_size", 0.25);
        config.segmentation.cluster_tolerance =
            declare_parameter<double>("cluster_tolerance", 0.65);
        config.segmentation.min_cluster_size =
            declare_parameter<int>("min_cluster_size", 8);
        config.segmentation.max_cluster_size =
            declare_parameter<int>("max_cluster_size", 250000);
        return config;
    }

    void handleCloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
        const std::vector<PointXYZ> input_points = cloudFromRosMessage(*message);
        const pointcloud_pipeline::PipelineResult result = pipeline_.process(input_points);

        processed_pub_->publish(cloudToRosMessage(result.downsampled_cloud, message->header));
        markers_pub_->publish(markersFromClusters(result.clusters, message->header));

        RCLCPP_DEBUG(get_logger(),
                     "Processed %zu -> %zu points, %zu clusters, total %.2f ms",
                     input_points.size(), result.downsampled_cloud.size(),
                     result.clusters.size(), result.timings.total_ms);
    }

    pointcloud_pipeline::PointCloudPipeline pipeline_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudPipelineNode>());
    rclcpp::shutdown();
    return 0;
}
