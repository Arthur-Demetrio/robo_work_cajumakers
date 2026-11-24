source /opt/ros/jazzy/setup.bash
source /home/marcos/robo_jazzy_ws/install_nav2_principal/setup.bash
source /home/marcos/Robo-WORK/install_retirada_fita_20261113/robo_virtual_wall/share/robo_virtual_wall/package.bash
export ROS_DOMAIN_ID=0
v4l2-ctl -d /dev/v4l/by-id/usb-XIFT_Web_Camera_20250904.1549-video-index0 --set-ctrl=focus_automatic_continuous=0 || exit 1
exec ros2 launch robo_virtual_wall observation_tape.launch.py device:=/dev/v4l/by-id/usb-XIFT_Web_Camera_20250904.1549-video-index0 homography_file:=/home/marcos/Robo-WORK/src/robo_virtual_wall/calibration/front_base_link_expanded_validated_20261113.json preview_homography_file:=/home/marcos/Robo-WORK/src/robo_virtual_wall/calibration/front_diagnostic_registered_20261113_212518.json
