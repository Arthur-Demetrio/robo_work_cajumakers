#!/usr/bin/env bash
set -e
# Localização / odometria
# Exige instalação ROS 2 Jazzy e workspace compilado no notebook do robô.
source /opt/ros/jazzy/setup.bash
source ~/robo_jazzy_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch robo_localization localization.launch.py
