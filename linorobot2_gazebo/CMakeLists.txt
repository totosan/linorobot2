cmake_minimum_required(VERSION 3.5)
project(linorobot2_gazebo)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS register)

add_library(g4_random_seed_system SHARED
  src/g4_random_seed_system.cpp
)
target_link_libraries(g4_random_seed_system
  ignition-gazebo6::ignition-gazebo6
  ignition-math6::ignition-math6
  ignition-plugin1::register
)
target_compile_features(g4_random_seed_system PRIVATE cxx_std_17)
install(TARGETS g4_random_seed_system
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY launch rviz worlds config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  scripts/camera_fault_proxy.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/linorobot2_gazebo.sh.in")
ament_package()