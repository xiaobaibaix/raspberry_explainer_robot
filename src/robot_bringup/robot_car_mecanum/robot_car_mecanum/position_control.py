#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import threading
import sys
import select
import termios
import tty
from geometry_msgs.msg import PoseStamped, Quaternion
from robot_msgs.action import ToPose
import tf_transformations


class KeyboardActionClient(Node):
    def __init__(self):
        super().__init__('keyboard_action_client')
        self._action_client = ActionClient(self, ToPose, '/position_tracking_controller/to_pose')

    def send_goal(self, x, y, theta_deg):
        goal_msg = ToPose.Goal()
        goal_msg.target_pose.header.frame_id = 'odom'
        goal_msg.target_pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.target_pose.pose.position.x = x
        goal_msg.target_pose.pose.position.y = y
        goal_msg.target_pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, theta_deg * 3.1415926 / 180.0)
        goal_msg.target_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.get_logger().info(f'Sending goal: x={x}, y={y}, theta={theta_deg}°')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        dist = fb.distance_remaining
        x=fb.current_pose.pose.position.x
        y=fb.current_pose.pose.position.y
        # 角度
        z=fb.current_pose.pose.orientation.z
        w=fb.current_pose.pose.orientation.w
        theta = tf_transformations.euler_from_quaternion([0,0,z,w])[2] * 180.0 / 3.1415926

        self.get_logger().info(f'Feedback: dist_remaining = {dist:.2f} m, x={x:.2f}, y={y:.2f}, theta={theta:.2f}°')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: success = {result.success}')
        self.prompt()          # 一轮结束，继续下一轮

    def prompt(self):
        print('\n=== 输入目标 (x y theta_degree) ===')
        print('（直接回车退出，发送后按 q 取消目标）')

    def cancel_goal(self):
        if hasattr(self, '_send_goal_future') and self._send_goal_future.done():
            goal_handle = self._send_goal_future.result()
            if goal_handle:
                future = goal_handle.cancel_goal_async()
                future.add_done_callback(lambda f: self.get_logger().info('Cancel requested'))


def kb_thread(node):
    node.prompt()
    while rclpy.ok():
        try:
            line = input().strip()
            if not line:
                break
            x, y, theta = map(float, line.split())
            node.send_goal(x, y, theta)
        except ValueError:
            print('格式:x y theta_degree')


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardActionClient()

    thread = threading.Thread(target=kb_thread, args=(node,), daemon=True)
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()