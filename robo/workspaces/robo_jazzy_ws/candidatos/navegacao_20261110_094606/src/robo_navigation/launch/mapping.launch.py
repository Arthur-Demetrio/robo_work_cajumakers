"""Inicia o slam_toolbox com a configuracao do robo real."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    navigation_share = get_package_share_directory("robo_navigation")
    slam_share = get_package_share_directory("slam_toolbox")

    slam_params_file = os.path.join(
        navigation_share, "config", "slam_mapping.yaml"
    )
    slam_launch_file = os.path.join(
        slam_share, "launch", "online_async_launch.py"
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Usar o relogio de simulacao",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch_file),
                launch_arguments={
                    "slam_params_file": slam_params_file,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
        ]
    )
