# lidar_mapper

ROS2 package for drone LIDAR 3D mapping.

## Dependencies
```bash
sudo apt install setserial
sudo apt install ros-jazzy-mavros
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
sudo apt install ros-jazzy-vrpn-mocap

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
* https://github.com/mavlink/mavros
* https://github.com/alvinsunyixiao/vrpn_mocap
