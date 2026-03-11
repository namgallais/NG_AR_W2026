cmake_minimum_required(VERSION 3.8)
project(dynamic_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(potential_field REQUIRED)
find_package(rclcpp REQUIRED)
find_package(highlevel_interfaces REQUIRED)

add_executable(dynamic_controller
  src/dynamic_controller.cpp
  src/DynamicController.cpp
)


target_include_directories(dynamic_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(dynamic_controller
  rclcpp
  sensor_msgs
  geometry_msgs
  potential_field
  highlevel_interfaces
)

target_link_libraries(dynamic_controller
  pinocchio::pinocchio
  Eigen3::Eigen
)



install(TARGETS dynamic_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

