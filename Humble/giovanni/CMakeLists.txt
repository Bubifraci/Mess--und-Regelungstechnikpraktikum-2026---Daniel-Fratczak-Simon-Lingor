cmake_minimum_required(VERSION 3.8)
project(giovanni)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(volksface REQUIRED)

include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../volksface/include
)

add_executable(gio-cont src/gio-cont.cc)
add_executable(gio_sim_path
  src/gio_sim_path.cc
  src/gio_path.cpp
)
add_executable(gio_volksbot
  src/gio_volksbot.cpp
  src/gio_path.cpp
)

ament_target_dependencies(gio-cont
  rclcpp
)

ament_target_dependencies(gio_sim_path
  rclcpp
)

ament_target_dependencies(gio_volksbot
  rclcpp
  geometry_msgs
  tf2
  volksface
)

install(TARGETS
  gio-cont
  gio_sim_path
  gio_volksbot
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()