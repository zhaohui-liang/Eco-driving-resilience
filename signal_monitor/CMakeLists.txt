cmake_minimum_required(VERSION 3.8)
project(signal_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
add_executable(signal_monitor_node src/signal_monitor_node.cpp)
ament_target_dependencies(signal_monitor_node rclcpp novatel_oem7_msgs)
install(TARGETS signal_monitor_node DESTINATION lib/${PROJECT_NAME})
# List all launch files manually or use file globbing
file(GLOB LAUNCH_FILES ${CMAKE_CURRENT_SOURCE_DIR}/launch/*.launch.py)

foreach(LAUNCH_FILE ${LAUNCH_FILES})
  get_filename_component(FILENAME ${LAUNCH_FILE} NAME)
  set(SRC ${LAUNCH_FILE})
  set(DST ${CMAKE_CURRENT_BINARY_DIR}/${FILENAME})

  execute_process(
    COMMAND ${CMAKE_COMMAND} -E create_symlink ${SRC} ${DST}
  )

  install(
    FILES ${DST}
    DESTINATION share/${PROJECT_NAME}/launch
  )
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
