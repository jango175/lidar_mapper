# lidar_mapper

ROS 2 package for drone LiDAR 3D mapping.

## Dependencies
```bash
sudo apt install setserial
sudo apt install ros-jazzy-mavros
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
sudo apt install ros-jazzy-mavros-extras
sudo apt install ros-jazzy-laser-geometry
sudo apt install ros-jazzy-vrpn-mocap
sudo apt install ros-jazzy-octomap-server
sudo apt install ros-jazzy-octomap-rviz-plugins
sudo apt install ros-jazzy-topic-tools
sudo apt install libomp-dev

cd services/
sudo cp lidar_mapper.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable lidar_mappper.service
sudo systemctl start lidar_mapper.service
```

You might also want to disable the power save in: `/etc/NetworkManager/conf.d/default-wifi-powersave-on.conf`.

## Build
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
source ./install/local_setup.bash
```

## Run
```bash
ros2 launch lidar_mapper lidar_mapper.launch.py
```
or
```bash
ros2 launch lidar_mapper lidar_mapper.launch.py use_optitrack:='true'
```
or just enable the service on the startup.

## Sources
* https://github.com/Myzhar/ldrobot-lidar-ros2
* https://github.com/mavlink/mavros
* https://github.com/OctoMap/octomap_mapping
* https://github.com/OctoMap/octomap_rviz_plugins
* https://github.com/pointcloudlibrary/pcl
* https://github.com/alvinsunyixiao/vrpn_mocap
