from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess, TimerAction
import os


def generate_launch_description():
  ldlidar_pkg = get_package_share_directory('ldlidar_node')
  ldlidar_node = IncludeLaunchDescription(
                  PythonLaunchDescriptionSource(
                    os.path.join(ldlidar_pkg, 'launch', 'ldlidar_with_mgr.launch.py')
    )
  )

  mavros_node = Node(
    package = 'mavros',
    executable = 'mavros_node',
    output = 'log',
    parameters = [{
      'fcu_url': '/dev/ttyS5:500000',
      'gcs_url': 'udp://@10.201.250.188:14550',
      'tgt_system': 1,
      'tgt_component': 1,
      'fcu_protocol': 'v2.0',
      'plugin_denylist': ['odometry']
    }]
  )

  lidar_mapper_node = Node(
    package = 'lidar_mapper',
    executable = 'lidar_mapper',
    name = 'lidar_mapper_node',
    output = 'screen',
    parameters = [
      {'lidar_mount_angle_deg': 30.0},
      {'mf_timeout': 0.25},
      {'timestamp_tolerance': 0.11}, # should be smaller than mf_timeout
      {'enable_bag': True}
    ]
  )

  octomap_node = Node(
    package = 'octomap_server',
    executable = 'octomap_server_node',
    name = 'octomap_server',
    output = 'log',
    parameters = [
      {'resolution': 0.15},
      {'frame_id': 'map'},
      {'base_frame_id': 'base_link'},
      {'sensor_model.max_range': 12.0},
      {'latch': True}
    ],
    remappings = [
      ('cloud_in', '/sync_point_cloud')
    ]
  )

  ld = LaunchDescription()
  ld.add_action(ldlidar_node)
  ld.add_action(mavros_node)
  ld.add_action(lidar_mapper_node)
  ld.add_action(octomap_node)

  return ld
