from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pointcloud_pipeline_ros",
            executable="pointcloud_pipeline_node",
            name="pointcloud_pipeline_node",
            output="screen",
            parameters=[{
                "input_topic": "/points_raw",
                "x_min": -80.0,
                "x_max": 80.0,
                "y_min": -80.0,
                "y_max": 80.0,
                "z_min": -3.0,
                "z_max": 5.0,
                "enable_statistical_outlier_removal": False,
                "enable_downsampling": True,
                "voxel_size": 0.25,
                "cluster_tolerance": 0.65,
                "min_cluster_size": 8,
                "max_cluster_size": 250000,
            }],
        )
    ])
