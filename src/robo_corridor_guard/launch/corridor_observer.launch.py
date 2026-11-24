from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robo_corridor_guard',executable='corridor_observer',
             name='robo_corridor_observer',output='screen',parameters=[{
                 'scan_topic':'/scan_front', 'base_frame':'base_link',
                 'robot_width_m':0.63,'min_width_m':0.70,'max_width_m':1.05,
                 'min_clearance_m':0.045,'max_scan_age_s':0.8,
                 'corridor_confirm_scans':3,'corridor_lost_scans':2
             }])
    ])
