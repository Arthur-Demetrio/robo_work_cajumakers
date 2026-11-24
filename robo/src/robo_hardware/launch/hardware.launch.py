from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("robo_hardware"))
    parameters = package_share / "config" / "hardware.yaml"

    return LaunchDescription(
        [
            Node(
                package="robo_hardware",
                executable="hardware_bridge_node",
                name="robo_hardware",
                output="screen",
                parameters=[str(parameters)],
            )
        ]
    )
