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

git submodule update --init --recursive

cd services/
sudo cp lidar_mapper.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable lidar_mappper.service
sudo systemctl start lidar_mapper.service
```

You might also want to disable the power save in: `/etc/NetworkManager/conf.d/default-wifi-powersave-on.conf`.

Change the IP addresses in `/launch/lidar_mapper.launch.py` accordingly.

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

This script can be paired with the `ego-planner-swarm` trajectory planner (!!! USE AT YOUR OWN RISK !!!).
```bash
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ros2 launch ego_planner ldlidar_advanced_param.launch.py
```

Send a waypoint for the `ego_planner` with:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{
  header: {frame_id: 'map'},
  pose: {
    position: {x: 4.0, y: 4.0, z: 3.0},
    orientation: {w: 1.0}
  }
}"
```

## Sources
* https://github.com/Myzhar/ldrobot-lidar-ros2
* https://github.com/mavlink/mavros
* https://github.com/OctoMap/octomap_mapping
* https://github.com/OctoMap/octomap_rviz_plugins
* https://github.com/pointcloudlibrary/pcl
* https://github.com/alvinsunyixiao/vrpn_mocap
* https://github.com/jango175/ego-planner-swarm
