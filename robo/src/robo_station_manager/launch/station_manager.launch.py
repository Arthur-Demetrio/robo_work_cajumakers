from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('allow_motion', default_value='false',
                              description='Permite go com confirmacao, DESLIGADO por padrao'),
        Node(
            package='robo_station_manager',
            executable='station_manager',
            name='station_manager',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'map_frame': 'map',
                'planner_id': 'GridBased',
                'robot_front_m': 0.365,
                'front_gap_m': 0.20,
                'enable_smoothing': True,
                'smoother_id': 'simple_smoother',
                'max_smoothing_duration_s': 3.0,
                'smoother_timeout_s': 10.0,
                'allow_motion': ParameterValue(LaunchConfiguration('allow_motion'), value_type=bool),
                'max_single_route_m': 15.0,
                'nav_timeout_s': 120.0,
                'goal_margin_m': 0.05,
            }],
        ),
    ])
