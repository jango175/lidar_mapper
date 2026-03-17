cmake_minimum_required(VERSION 3.8)
project(lidar_mapper)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(NODE_NAME lidar_mapper)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_subdirectory(lib/ws2812b_control)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(${NODE_NAME} src/lidar_mapper.cpp) 
ament_target_dependencies(${NODE_NAME}
  rclcpp
  message_filters
  mavros_msgs
  sensor_msgs
  geometry_msgs
  rosbag2_cpp
  laser_geometry
  tf2_ros
)

target_link_libraries(${NODE_NAME} ws2812b_control)

install(TARGETS
  ${NODE_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
