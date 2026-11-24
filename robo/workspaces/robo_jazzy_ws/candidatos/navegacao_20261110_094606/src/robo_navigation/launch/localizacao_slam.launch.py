"""LOCALIZACAO_FIXA (09/11): slam_toolbox em modo localizacao (o mapa nao muda na missao)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    params = os.path.join(get_package_share_directory("robo_navigation"),
                          "config", "slam_localizacao.yaml")
    loc = os.path.join(get_package_share_directory("slam_toolbox"),
                       "launch", "localization_launch.py")
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(loc),
            launch_arguments={"slam_params_file": params, "use_sim_time": "false",
                              "autostart": "true"}.items()),
    ])
