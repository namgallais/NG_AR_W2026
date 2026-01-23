cmake_minimum_required(VERSION 3.8)
project(keyboard_controller)

# Compiler warnings (optional but standard)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include headers
include_directories(include)

# Build executable
add_executable(keyboard_controller_node
  src/KeyboardController.cpp
  src/keyboard_controller_node.cpp
)

ament_target_dependencies(
  keyboard_controller_node
  rclcpp
  std_msgs
  geometry_msgs
)

# Install executable
install(TARGETS
  keyboard_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include
)

ament_package()

