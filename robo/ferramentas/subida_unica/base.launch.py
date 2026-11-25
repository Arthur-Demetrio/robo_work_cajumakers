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

    # SEM_PAREDE_VIRTUAL (23/11): parede virtual (camera XIFT + tape_keepout) removida.
    # MAPA_LIMPO (21/11): /map -> /map_nav para o costmap global (sem zonas no mapa novo).
    actions.append(ExecuteProcess(name="mapa_limpo", output="screen", respawn=True, respawn_delay=2.0,
                                  cmd=["python3", "-u", str(home / "robo_jazzy_ws/ferramentas/subida_unica/mapa_limpo.py")]))
    # SLAM_IMU_FORTE / SEM_MAPA_NO_GIRO (23/11): o SLAM le /scan_slam (scan sem giro).
    actions.append(ExecuteProcess(name="scan_sem_giro", output="screen", respawn=True, respawn_delay=1.0,
                                  cmd=["python3", "-u", str(home / "robo_jazzy_ws/ferramentas/subida_unica/scan_sem_giro.py")]))
    return LaunchDescription(actions)
