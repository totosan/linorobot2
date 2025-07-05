cmake_minimum_required(VERSION 3.5)
project(linorobot2_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/lidar_monitor.py scripts/check_lidar.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
