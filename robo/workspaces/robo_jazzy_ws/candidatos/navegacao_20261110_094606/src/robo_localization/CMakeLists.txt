cmake_minimum_required(VERSION 3.8)
project(robo_localization)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS scripts/command_imu_odometry_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME command_imu_odometry_node
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
