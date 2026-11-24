cmake_minimum_required(VERSION 3.8)
project(robo_lidar)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/scan_filter_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME scan_filter_node
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
