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

    msp_node = Node(
        package = 'msp_controller_ros2',
        executable = 'msp_publisher',
        name = 'msp_publisher_node'
    )

    ldlidar_sub_node = Node(
        package = 'ldlidar_sub',
        executable = 'lidar_subscriber',
        name = 'ldlidar_sub_node'
    )

    ld = LaunchDescription()
    ld.add_action(ldlidar_node)
    ld.add_action(msp_node)
    ld.add_action(ldlidar_sub_node)

    return ld