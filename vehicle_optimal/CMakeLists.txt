cmake_minimum_required(VERSION 3.8)
project(vehicle_optimal)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(casadi REQUIRED)


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

# Define executable target
add_executable(vehicle_optimal_node ${SOURCES})

# Link dependencies
ament_target_dependencies(vehicle_optimal_node
  rclcpp
  std_msgs
  geometry_msgs
  novatel_oem7_msgs
  casadi
)

# Link libraries (direct linking)
target_link_libraries(vehicle_optimal_node
  casadi
)

# Install target executable
install(TARGETS vehicle_optimal_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

# Linting (optional testing support)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
