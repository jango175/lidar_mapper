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
    output = 'screen',
    parameters = [{
      'fcu_url': '/dev/ttyS5:500000',
      'gcs_url': 'udp://@192.168.0.144:14550',
      'tgt_system': 1,
      'tgt_component': 1,
      'fcu_protocol': 'v2.0'
    }]
  )

  set_stream_rate = ExecuteProcess(
    cmd = [
      'ros2', 'service', 'call',
      '/mavros/set_stream_rate', 'mavros_msgs/srv/StreamRate',
      '{stream_id: 0, message_rate: 10, on_off: true}'
    ],
    output = 'screen'
  )

  delayed_stream_request = TimerAction(
    period = 20.0,
    actions = [set_stream_rate]
  )

  lidar_mapper_node = Node(
    package = 'lidar_mapper',
    executable = 'lidar_mapper',
    name = 'lidar_mapper_node',
    parameters = [
      {'timestamp_diff_threshold': 0.025},
      {'enable_bag': True},
    ]
  )

  ld = LaunchDescription()
  ld.add_action(ldlidar_node)
  ld.add_action(mavros_node)
  ld.add_action(delayed_stream_request)
  ld.add_action(lidar_mapper_node)

  return ld
