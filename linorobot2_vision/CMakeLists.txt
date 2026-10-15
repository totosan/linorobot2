cmake_minimum_required(VERSION 3.5)
project(linorobot2_vision)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/python/${PROJECT_NAME}"
)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/fixtures)
file(GLOB VISION_FIXTURE_FILES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/fixtures/*
)
foreach(FIXTURE_FILE ${VISION_FIXTURE_FILES})
  get_filename_component(FIXTURE_NAME ${FIXTURE_FILE} NAME)
  configure_file(
    ${FIXTURE_FILE}
    ${CMAKE_CURRENT_BINARY_DIR}/fixtures/${FIXTURE_NAME}
    COPYONLY
  )
endforeach()
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/fixtures/
  DESTINATION share/${PROJECT_NAME}/fixtures
)
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/scripts)
file(COPY
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/vision_adapter_node
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/vision_image_overlay_node
  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/vision_lidar_fusion_node
  DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/scripts
  FILE_PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)
install(
  PROGRAMS
    ${CMAKE_CURRENT_BINARY_DIR}/scripts/vision_adapter_node
    ${CMAKE_CURRENT_BINARY_DIR}/scripts/vision_image_overlay_node
    ${CMAKE_CURRENT_BINARY_DIR}/scripts/vision_lidar_fusion_node
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()