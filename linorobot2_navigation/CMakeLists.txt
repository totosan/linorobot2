cmake_minimum_required(VERSION 3.5)
project(linorobot2_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz maps behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

# Keep runtime Nav2 YAML inputs inside the Linux build tree. The workspace is
# mounted from macOS in SIL; symlinking runtime inputs back across that mount
# can produce transient ENOENT errors during a Nav2 launch.
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/config)
file(GLOB NAVIGATION_CONFIG_FILES CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/config/*.yaml
)
foreach(CONFIG_FILE ${NAVIGATION_CONFIG_FILES})
  get_filename_component(CONFIG_NAME ${CONFIG_FILE} NAME)
  configure_file(
    ${CONFIG_FILE}
    ${CMAKE_CURRENT_BINARY_DIR}/config/${CONFIG_NAME}
    COPYONLY
  )
endforeach()
install(
  DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
