# MAPA_NOVO (19/11): SLAM do zero; sem deserialize_map da arena e sem mesas da arena.
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    home = Path.home()
    actions = []
    for package, filename in [
        ("robo_hardware", "hardware.launch.py"),
        ("robo_lidar", "lidar_filter.launch.py"),
        ("robo_localization", "localization.launch.py"),
        ("robo_navigation", "mapping.launch.py"),
    ]:
        launch_file = Path(get_package_share_directory(package)) / "launch" / filename
        actions.append(GroupAction(actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_file))
        )]))

    device = "/dev/v4l/by-id/usb-XIFT_Web_Camera_20250904.1549-video-index0"
    wall = str(home / "logs/reabertura_20261116_100105/parede.sh")
    camera_command = ('if [[ ! -e "$1" ]]; then echo "Aguardando a câmera frontal XIFT"; fi; '
                      'while [[ ! -e "$1" ]]; do sleep 2; done; exec bash "$2"')
    for name, command in [
        ("parede_virtual", ["bash", "-c", camera_command, "camera", device, wall]),
    ]:
        actions.append(ExecuteProcess(name=name, cmd=command, output="screen"))
    return LaunchDescription(actions)
