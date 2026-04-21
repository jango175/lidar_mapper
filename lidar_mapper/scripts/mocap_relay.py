#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from rclpy.node import Publisher, Subscription
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped


class MocapRelay(Node):
  def __init__(self, rigid_body_name: str):
    super().__init__(f'mocap_relay_{rigid_body_name}')

    # VRPN QoS: Best Effort / Volatile
    vrpn_qos = QoSProfile(
      depth=10,
      reliability=ReliabilityPolicy.BEST_EFFORT,
      durability=DurabilityPolicy.VOLATILE
    )

    # MAVROS QoS: Reliable / Volatile
    mavros_qos = QoSProfile(
      depth=10,
      reliability=ReliabilityPolicy.RELIABLE,
      durability=DurabilityPolicy.VOLATILE
    )

    vrpn_topic = f'/vrpn_mocap/{rigid_body_name}/pose'
    self.get_logger().info(f'Subscribing to {vrpn_topic} and relaying to /mavros/vision_pose/pose')

    self.sub: Subscription = self.create_subscription(
      PoseStamped,
      vrpn_topic,
      self.pose_callback,
      vrpn_qos
    )

    self.pub: Publisher = self.create_publisher(
      PoseStamped,
      '/mavros/vision_pose/pose',
      mavros_qos
    )


  def pose_callback(self, msg: PoseStamped):
    msg.header.frame_id = 'map'
    self.pub.publish(msg)


def main(args = None):
  # Check if the rigid body name was passed as an argument
  if len(sys.argv) < 2:
    print("Error: You must provide a rigid_body_name as an argument.")
    sys.exit(1)

  rigid_body_name = sys.argv[1]

  rclpy.init(args = args)
  node = MocapRelay(rigid_body_name)
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


# Entry point
if __name__ == '__main__':
  main()
