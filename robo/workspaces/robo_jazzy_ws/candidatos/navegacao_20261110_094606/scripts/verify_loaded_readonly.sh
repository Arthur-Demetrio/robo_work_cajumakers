#!/usr/bin/env bash
# Run in the ROS environment/domain being inspected. Read-only; no lifecycle/goals.
set -euo pipefail
nav_ns="${1:-}"
nav_ns="${nav_ns%/}"
ros2 param get "$nav_ns/controller_server" FollowPath.plugin
ros2 param get "$nav_ns/controller_server" FollowPath.sim_time
ros2 param get "$nav_ns/controller_server" FollowPath.GoalAlign.scale
ros2 param get "$nav_ns/bt_navigator" default_nav_to_pose_bt_xml
ros2 param get "$nav_ns/global_costmap/global_costmap" plugins
ros2 param get "$nav_ns/global_costmap/global_costmap" obstacle_layer.scan.topic
ros2 param get "$nav_ns/global_costmap/global_costmap" obstacle_layer.scan.marking
ros2 param get "$nav_ns/global_costmap/global_costmap" obstacle_layer.scan.obstacle_max_range
ros2 topic info "$nav_ns/scan_nav" --verbose
ros2 action info "$nav_ns/supervised_escape"
