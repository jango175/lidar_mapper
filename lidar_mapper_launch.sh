#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash

ros2 launch lidar_mapper lidar_mapper.launch.py
