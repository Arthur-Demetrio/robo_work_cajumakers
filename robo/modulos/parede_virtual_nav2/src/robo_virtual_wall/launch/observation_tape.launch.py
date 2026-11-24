"""Reload only tape perception/keepout, leaving the current Nav2 and base alive."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    from robo_virtual_wall.projection import load_calibration
    value = lambda name: LaunchConfiguration(name).perform(context)
    calibration = Path(value('homography_file')).expanduser()
    load_calibration(calibration)
    return [Node(package='robo_virtual_wall', executable='virtual_wall', name='virtual_wall',
        output='screen', parameters=[dict(homography_file=str(calibration),
            preview_homography_file=value('preview_homography_file'), width=640, height=480,
            roi_top=0., roi_bottom=.85,
            show_windows=ParameterValue(LaunchConfiguration('show_windows'),value_type=bool),
            device=value('device'))]),
        Node(package='robo_virtual_wall', executable='tape_keepout', name='tape_keepout',
            output='screen', parameters=[dict(remove_visible_missing=True)])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('homography_file'),
        DeclareLaunchArgument('preview_homography_file',default_value=''),
        DeclareLaunchArgument('show_windows',default_value='true'),
        DeclareLaunchArgument('device',default_value='/dev/video2'),
        OpaqueFunction(function=start)])
