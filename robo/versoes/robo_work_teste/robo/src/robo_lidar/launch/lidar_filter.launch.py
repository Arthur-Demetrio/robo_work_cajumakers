from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("robo_lidar"),
        "config",
        "lidar.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="robo_lidar",
                executable="scan_filter_node",
                name="scan_filter",
                output="screen",
                parameters=[config],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser_tf",
                output="screen",
                arguments=[
                    "--x", "0.2755",
                    "--y", "0.005",
                    "--z", "0.075",
                    "--roll", "0.0",
                    "--pitch", "0.0",
                    "--yaw", "0.0",
                    "--frame-id", "base_link",
                    "--child-frame-id", "laser_raw",
                ],
            ),
        ]
    )
