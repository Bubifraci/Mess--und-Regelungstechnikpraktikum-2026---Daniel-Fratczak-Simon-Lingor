cmake_minimum_required(VERSION 3.8)
project(odom_path)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(path_gen src/path_gen.cpp)

ament_target_dependencies(
  path_gen
  rclcpp
  std_msgs
)

install(TARGETS
  path_gen
  DESTINATION lib/${PROJECT_NAME})

ament_package()