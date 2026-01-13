#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import argparse

class TwistStamperNode(Node):
    def __init__(self, in_topic, out_topic, frame_id):
        super().__init__('twist_stamper')
        self._frame_id = frame_id
        self._pub = self.create_publisher(TwistStamped, out_topic, 10)
        self._sub = self.create_subscription(
            Twist, in_topic, self._callback, 10)
        self.get_logger().info(
            f'Relaying Twist("{in_topic}") -> TwistStamped("{out_topic}") '
            f'with frame_id="{frame_id}"')

    def _callback(self, msg):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = self._frame_id
        stamped.twist = msg
        self._pub.publish(stamped)

def main():
    rclpy.init()

    node = TwistStamperNode('cmd_vel_smoothed',
                             'mecanum_drive_controller/reference',
                             'base_link')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()