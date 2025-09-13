cmake_minimum_required(VERSION 3.8)
project(ldlidar_sub)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

add_executable(lidar_subscriber src/subscriber_lambda_function.cpp)
ament_target_dependencies(lidar_subscriber rclcpp std_msgs sensor_msgs geometry_msgs rosbag2_cpp)

install(TARGETS
  lidar_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

# install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
