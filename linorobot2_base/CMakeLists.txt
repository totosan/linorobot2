cmake_minimum_required(VERSION 3.5)
project(linorobot2_base)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/command_timeout.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
