"""Nav2 plus calibrated tape vision; requires a validated metric calibration."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def start(context):
    from pathlib import Path
    from robo_virtual_wall.projection import load_calibration
    calibration = Path(LaunchConfiguration('homography_file').perform(context)).expanduser()
    load_calibration(calibration)  # fail before starting navigation with a preview file
    navigation = get_package_share_directory('robo_navigation')
    vision = get_package_share_directory('robo_virtual_wall')
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(vision,'launch','vision.launch.py')),
            launch_arguments=dict(homography_file=str(calibration),
                device=LaunchConfiguration('device'),
                image_topic=LaunchConfiguration('image_topic'),
                show_windows=LaunchConfiguration('show_windows'),roi_top='0.0',
                use_sim_time=LaunchConfiguration('use_sim_time')).items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(navigation,'launch','navigation.launch.py')),
            launch_arguments=dict(virtual_walls='true',
                use_sim_time=LaunchConfiguration('use_sim_time')).items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('homography_file',description='Validated raw-image base_link calibration'),
        DeclareLaunchArgument('device',default_value='/dev/video2'),
        DeclareLaunchArgument('image_topic',default_value=''),
        DeclareLaunchArgument('show_windows',default_value='true'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        OpaqueFunction(function=start),
    ])
