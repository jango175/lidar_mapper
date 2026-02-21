# lidar_mapper

ROS2 package for drone LIDAR 3D mapping.

## Dependencies
```bash
git submodule update --init --recursive
sudo apt install setserial
cd services/
sudo cp lidar_mapper.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable lidar_mappper.service
sudo systemctl start lidar_mapper.service
```

## Build
```bash
cd ~/ros2_ws
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
source ./install/local_setup.bash
```

## Sources
* https://github.com/Myzhar/ldrobot-lidar-ros2
* https://github.com/jango175/msp_controller_ros2
