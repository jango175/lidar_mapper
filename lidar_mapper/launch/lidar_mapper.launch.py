from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
  ldlidar_pkg = get_package_share_directory('ldlidar_node')
  ldlidar_node = IncludeLaunchDescription(
                  PythonLaunchDescriptionSource(
                    os.path.join(ldlidar_pkg, 'launch', 'ldlidar_with_mgr.launch.py')
    )
  )

  msp_pkg = get_package_share_directory('msp_controller_ros2')
  msp_node = IncludeLaunchDescription(
                  PythonLaunchDescriptionSource(
                    os.path.join(msp_pkg, 'launch', 'msp_publisher.launch.py')
    )
  )

  lidar_mapper_node = Node(
    package = 'lidar_mapper',
    executable = 'lidar_mapper',
    name = 'lidar_mapper_node',
    parameters = [
      {'timestamp_diff_threshold': 0.025},
      {'enable_bag': True},
      {'enable_log': True},
      {'use_ned': True}
    ]
  )

  ld = LaunchDescription()
  ld.add_action(ldlidar_node)
  ld.add_action(msp_node)
  ld.add_action(lidar_mapper_node)

  return ld
