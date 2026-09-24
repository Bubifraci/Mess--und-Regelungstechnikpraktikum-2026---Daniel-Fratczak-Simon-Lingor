cmake_minimum_required(VERSION 3.8)
project(path_logger)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Executable hinzufügen
add_executable(logger src/path_logger_node.cpp)
ament_target_dependencies(logger rclcpp nav_msgs std_msgs)

# Installation definieren
install(TARGETS
  logger
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
