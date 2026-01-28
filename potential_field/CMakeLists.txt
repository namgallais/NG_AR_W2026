cmake_minimum_required(VERSION 3.8)
project(potential_field)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(highlevel_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(potential_field_2d
  src/potential_field_2d.cpp
  src/PotentialField.cpp
)


target_include_directories(potential_field_2d PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(potential_field_2d
  rclcpp
  std_msgs
  geometry_msgs
  highlevel_interfaces
)

target_link_libraries(potential_field_2d
  Eigen3::Eigen
)

install(TARGETS potential_field_2d
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include
  DESTINATION include
)

ament_package()

