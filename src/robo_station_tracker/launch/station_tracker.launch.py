from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    path=os.path.join(get_package_share_directory('robo_station_tracker'),'config','tracker.yaml')
    return LaunchDescription([Node(package='robo_station_tracker',executable='station_tracker',
                                   name='robo_station_tracker',output='screen',parameters=[path])])
