#!/usr/bin/env bash
set -e
# Zera a referência de odometria na pose atual.
# Requer o nó de localização ativo (terminal 03).
source /opt/ros/jazzy/setup.bash
source ~/robo_jazzy_ws/install/setup.bash
export ROS_DOMAIN_ID=0

ros2 service call /reset_odometry std_srvs/srv/Trigger "{}"
