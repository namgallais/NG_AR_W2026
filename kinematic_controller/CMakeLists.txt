cmake_minimum_required(VERSION 3.8)
project(kinematic_controller)

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

add_executable(kinematic_controller
  src/kinematic_controller.cpp
  src/KinematicController.cpp
)


target_include_directories(kinematic_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(kinematic_controller
  rclcpp
  sensor_msgs
  geometry_msgs
  potential_field
)

target_link_libraries(kinematic_controller
  pinocchio::pinocchio
  Eigen3::Eigen
)



install(TARGETS kinematic_controller
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

