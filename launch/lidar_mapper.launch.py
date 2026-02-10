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
      'gcs_url': 'udp://@10.234.177.188:14550',
      'tgt_system': 1,
      'tgt_component': 1,
      'fcu_protocol': 'v2.0'
    }]
  )

  req_base = ExecuteProcess(
    cmd = [
      'ros2', 'service', 'call',
      '/mavros/set_stream_rate', 'mavros_msgs/srv/StreamRate',
      '{stream_id: 0, message_rate: 10, on_off: true}'
    ],
    output = 'log'
  )

  req_rc = ExecuteProcess(
    cmd = [
      'ros2', 'service', 'call',
      '/mavros/set_stream_rate', 'mavros_msgs/srv/StreamRate',
      '{stream_id: 3, message_rate: 20, on_off: true}'
    ],
    output = 'log'
  )

  req_pos_gps = ExecuteProcess(
    cmd = [
      'ros2', 'service', 'call',
      '/mavros/set_stream_rate', 'mavros_msgs/srv/StreamRate',
      '{stream_id: 6, message_rate: 20, on_off: true}'
    ],
    output = 'log'
  )

  req_imu = ExecuteProcess(
    cmd = [
      'ros2', 'service', 'call',
      '/mavros/set_stream_rate', 'mavros_msgs/srv/StreamRate',
      '{stream_id: 10, message_rate: 50, on_off: true}'
    ],
    output = 'log'
  )

  delayed_base_request = TimerAction(
    period = 20.0,
    actions = [req_base]
  )

  delayed_sensors_request = TimerAction(
    period = 25.0,
    actions = [req_rc, req_pos_gps, req_imu]
  )

  lidar_mapper_node = Node(
    package = 'lidar_mapper',
    executable = 'lidar_mapper',
    name = 'lidar_mapper_node',
    output = 'screen',
    parameters = [
      {'timestamp_diff_threshold': 0.025},
      {'enable_bag': True},
    ]
  )

  ld = LaunchDescription()
  ld.add_action(ldlidar_node)
  ld.add_action(mavros_node)
  ld.add_action(delayed_base_request)
  ld.add_action(delayed_sensors_request)
  ld.add_action(lidar_mapper_node)

  return ld
