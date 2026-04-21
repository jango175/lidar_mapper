import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext, Action
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup_nodes(context: LaunchContext, *_args: object, **_kwargs: object):
  world_link = LaunchConfiguration('world_link').perform(context)
  drone_link = LaunchConfiguration('drone_link').perform(context)
  lidar_link = LaunchConfiguration('lidar_link').perform(context)

  use_optitrack = LaunchConfiguration('use_optitrack').perform(context).lower() == 'true'
  rigid_body_name = LaunchConfiguration('rigid_body_name').perform(context)

  ip_address = 'udp://@10.49.187.188:14550' # Hotspot
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
      'lidar_mount_roll_deg': 0.0,
      'lidar_mount_pitch_deg': 30.0,
      'lidar_mount_yaw_deg': 0.0,
      'lidar_mount_offset_x': 0.088,
      'lidar_mount_offset_y': 0.0,
      'lidar_mount_offset_z': 0.088,
      'world_link': world_link,
      'drone_link': drone_link,
      'lidar_link': lidar_link,
      'mf_timeout': 0.25,
      'timestamp_tolerance': 0.11, # should be smaller than mf_timeout
      'window_size': 20.0,
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
      'frame_id': world_link,
      'base_frame_id': drone_link,
      'sensor_model.max_range': 12.0,
      'latch': True
    }],
    remappings = [
      ('cloud_in', '/lidar_mapper/sync_slice_point_cloud')
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

    mocap_relay_node = Node(
      package = 'lidar_mapper',
      executable = 'mocap_relay.py',
      name = 'mocap_relay_node',
      output = 'screen',
      arguments = [rigid_body_name]
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
    nodes_to_launch.append(mocap_relay_node)
    nodes_to_launch.append(set_origin_cmd)

  return nodes_to_launch


def generate_launch_description():
  world_link_arg = DeclareLaunchArgument(
    'world_link',
    default_value = 'map',
    description = 'World link name'
  )

  drone_link_arg = DeclareLaunchArgument(
    'drone_link',
    default_value = 'base_link',
    description = 'Drone link name'
  )

  lidar_link_arg = DeclareLaunchArgument(
    'lidar_link',
    default_value = 'ldlidar_link',
    description = 'LIDAR link name'
  )

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
    world_link_arg,
    drone_link_arg,
    lidar_link_arg,
    use_optitrack_arg,
    rigid_body_arg,
    OpaqueFunction(function = setup_nodes)
  ])
