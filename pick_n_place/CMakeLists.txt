cmake_minimum_required(VERSION 3.8)
project(pick_n_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(highlevel_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(pick_n_place
  src/pick_n_place.cpp
  src/PickNPlace.cpp
)


target_include_directories(pick_n_place PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(pick_n_place
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  highlevel_interfaces
  control_msgs
)

target_link_libraries(pick_n_place
  Eigen3::Eigen
)


install(TARGETS pick_n_place
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

