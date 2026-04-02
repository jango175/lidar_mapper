import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext, Action
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup_nodes(context: LaunchContext, *_args: object, **_kwargs: object):
  use_optitrack = LaunchConfiguration('use_optitrack').perform(context).lower() == 'true'
  rigid_body_name = LaunchConfiguration('rigid_body_name').perform(context)

  ip_address = 'udp://@10.48.59.188:14550' # Hotspot
  if use_optitrack:
    ip_address = 'udp://@192.168.16.63:14550' # OptiTrack

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
      'plugin_denylist': ['*'],
      'plugin_allowlist': [
        'sys_status',
        'sys_time',
        'vision_pose',
        'vision_speed',
        'global_position',
        'local_position',
        'command',
        'imu',
        'rc_io',
        'mocap_pose_estimate',
        'optical_flow',
        'rangefinder',
        'setpoint_accel',
        'setpoint_attitude',
        'setpoint_position',
        'setpoint_raw',
        'setpoint_trajectory',
        'setpoint_velocity'
      ]
    }]
  )

  lidar_mapper_node = Node(
    package = 'lidar_mapper',
    executable = 'lidar_mapper',
    name = 'lidar_mapper_node',
    output = 'screen',
    parameters = [{
      'lidar_mount_angle_deg': 30.0,
      'mf_timeout': 0.25,
      'timestamp_tolerance': 0.11, # should be smaller than mf_timeout
      'sor_mean_k': 50,
      'sor_std_dev_mult': 1.0,
      'enable_bag': True
    }]
  )

  octomap_node = Node(
    package = 'octomap_server',
    executable = 'octomap_server_node',
    name = 'octomap_server',
    output = 'log',
    parameters = [{
      'resolution': 0.15,
      'frame_id': 'map',
      'base_frame_id': 'base_link',
      'sensor_model.max_range': 12.0,
      'latch': True
    }],
    remappings = [
      ('cloud_in', '/sync_slice_point_cloud')
    ]
  )

  nodes_to_launch: list[Action] = [ldlidar_node, mavros_node, lidar_mapper_node, octomap_node]

  if use_optitrack:
    vrpn_mocap_pkg = get_package_share_directory('vrpn_mocap')
    vrpn_node = IncludeLaunchDescription(
      AnyLaunchDescriptionSource(
        os.path.join(vrpn_mocap_pkg, 'launch', 'client.launch.yaml')
      ),
      launch_arguments = [
        ('server', '192.168.16.50'),
        ('port', '3883'),
      ]
    )

    relay_node = Node(
      package = 'topic_tools',
      executable = 'relay',
      name = 'mocap_relay_node',
      output = 'log',
      remappings = [
        ('input', f'/vrpn_mocap/{rigid_body_name}/pose'),
        ('output', '/mavros/vision_pose/pose')
      ]
    )

    set_origin_cmd = TimerAction(
      period = 10.0,
      actions = [
        ExecuteProcess(
          cmd = [
            'ros2', 'topic', 'pub', '--once',
            '/mavros/global_position/set_gp_origin',
            'geographic_msgs/msg/GeoPointStamped',
            '"{header: {frame_id: \'map\'}, position: {latitude: 55.470368, longitude: 10.329439, altitude: 15.0}}"'
          ],
          output = 'log'
        )
      ]
    )

    nodes_to_launch.append(vrpn_node)
    nodes_to_launch.append(relay_node)
    nodes_to_launch.append(set_origin_cmd)

  return nodes_to_launch


def generate_launch_description():
  use_optitrack_arg = DeclareLaunchArgument(
    'use_optitrack',
    default_value = 'false',
    description = 'Set true when using OptiTrack system'
  )

  rigid_body_arg = DeclareLaunchArgument(
    'rigid_body_name',
    default_value = 'lidar_drone',
    description = 'Name of the VRPN rigid body'
  )

  return LaunchDescription([
    use_optitrack_arg,
    rigid_body_arg,
    OpaqueFunction(function = setup_nodes)
  ])
