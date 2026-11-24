import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("robo_lidar_odometry")
    config = os.path.join(share, "config", "lidar_odometry.yaml")
    return LaunchDescription(
        [
            Node(
                package="robo_lidar_odometry",
                executable="lidar_odometry_node",
                name="odometry_node",
                output="screen",
                parameters=[config, {"use_sim_time": False}],
            )
        ]
    )
