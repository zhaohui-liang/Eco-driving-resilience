cmake_minimum_required(VERSION 3.8)
project(vehicle_optimal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(osqp REQUIRED)
find_package(osqp-eigen REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
target_link_libraries(${PROJECT_NAME}
  PRIVATE
    Eigen3::Eigen
    osqp::osqp
    osqp::osqp-eigen
)

# Include directories
include_directories(
  include
)

# Source files
set(SOURCES
  src/vehicle_node.cpp
  src/vehicle_optimal.cpp
  src/signal_io.cpp
)

# Executable
add_executable(vehicle_optimal_node ${SOURCES})
ament_target_dependencies(vehicle_optimal_node
  rclcpp
  std_msgs
  geometry_msgs
  novatel_oem7_msgs
)

target_link_libraries(vehicle_optimal_node
  ${catkin_LIBRARIES}
)

# Install targets
install(TARGETS vehicle_controller_node
  DESTINATION lib/${PROJECT_NAME})

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

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)


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
