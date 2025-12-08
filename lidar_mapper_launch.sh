#!/bin/bash

source /opt/ros/jazzy/setup.bash
source /home/orangepi/ros2_ws/install/setup.bash

/usr/bin/setserial /dev/ttyS5 low_latency
/usr/bin/setserial /dev/ttyUSB0 low_latency
echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer

ros2 launch lidar_mapper lidar_mapper.launch.py
