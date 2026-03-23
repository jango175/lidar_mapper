from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
import os


def generate_launch_description():
  use_optitrack_arg = DeclareLaunchArgument(
    'use_optitrack', default_value='false',
    description='Set true when using OptiTrack system'
  )

  rigid_body_arg = DeclareLaunchArgument(
    'rigid_body_name', default_value='lidar_drone',
    description='Name of the VRPN rigid body'
  )

  use_optitrack = LaunchConfiguration('use_optitrack')
  rigid_body_name = LaunchConfiguration('rigid_body_name')
  ip_address = PythonExpression([
    "'udp://@192.168.16.63:14550' if '", use_optitrack, "' == 'true' else 'udp://@10.48.59.188:14550'"
  ])

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
      'gcs_url': ip_address,
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

  vrpn_mocap_pkg = get_package_share_directory('vrpn_mocap')
  vrpn_node = IncludeLaunchDescription(
    AnyLaunchDescriptionSource(
      os.path.join(vrpn_mocap_pkg, 'launch', 'client.launch.yaml')
    ),
    launch_arguments={
      'server': '192.168.16.50',
      'port': '3883',
    }.items(),
    condition=IfCondition(use_optitrack)
  )

  relay_node = Node(
    package='topic_tools',
    executable='relay',
    name='mocap_relay_node',
    output='screen',
    arguments=[
      ['/vrpn_mocap/', LaunchConfiguration('rigid_body_name'), '/pose'], 
      '/mavros/vision_pose/pose'
    ],
    condition=IfCondition(use_optitrack)
  )

  ld = LaunchDescription()
  ld.add_action(ldlidar_node)
  ld.add_action(mavros_node)
  ld.add_action(lidar_mapper_node)
  ld.add_action(octomap_node)
  ld.add_action(vrpn_node)
  ld.add_action(relay_node)

  return ld
