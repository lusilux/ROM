cmake_minimum_required(VERSION 3.8)
project(practica9_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(nav_msgs REQUIRED)

find_package(laser_geometry)
find_package(sensor_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

add_executable(context_label_publisher src/context_label_publisher.cpp)
ament_target_dependencies(
  context_label_publisher 
  rclcpp 
  std_msgs
  sensor_msgs
  laser_geometry
  tf2_sensor_msgs
  nav_msgs
)

add_executable(context_speed_limiter src/context_speed_limiter.cpp)
ament_target_dependencies(
  context_speed_limiter 
  rclcpp 
  std_msgs 
  rcl_interfaces
)



install(TARGETS
  context_label_publisher
  context_speed_limiter
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch param
DESTINATION share/${PROJECT_NAME})

ament_package()
