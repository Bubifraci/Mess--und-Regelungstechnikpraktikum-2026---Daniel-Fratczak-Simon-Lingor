cmake_minimum_required(VERSION 3.8)
project(my_path_generator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(odom_path_generator src/odom_path_generator.cpp)
add_executable(amcl_path_generator src/amcl_path_generator.cpp)

ament_target_dependencies(odom_path_generator
  rclcpp
  nav_msgs
)

ament_target_dependencies(amcl_path_generator
  rclcpp
  geometry_msgs
  tf2
)

install(TARGETS
  odom_path_generator
  amcl_path_generator
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
