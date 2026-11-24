from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("robo_localization"))
    parameters = package_share / "config" / "localization.yaml"

    return LaunchDescription(
        [
            Node(
                package="robo_localization",
                executable="command_imu_odometry_node",
                name="command_imu_odometry",
                output="screen",
                parameters=[str(parameters)],
            )
        ]
    )
