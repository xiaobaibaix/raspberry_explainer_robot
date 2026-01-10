#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage

class TfOdomToTf(Node):
    def __init__(self):
        super().__init__('tf_odom_to_tf_node')
        # 订阅TFMessage类型的话题
        self.sub = self.create_subscription(
            TFMessage,
            '/mecanum_drive_controller/tf_odometry',  # 目标话题
            self.tf_callback,
            10  # 队列大小
        )
        # TF广播器（发布到TF树）
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info("TF转发节点已启动，订阅话题: /mecanum_drive_controller/tf_odometry")

    def tf_callback(self, msg: TFMessage):
        """解析TFMessage并转发到TF树"""
        # TFMessage包含一个transforms列表（可能有多个TF变换）
        if not msg.transforms:
            self.get_logger().warn("收到空的TFMessage，跳过发布")
            return
        
        # 遍历所有TF变换（通常只有1个）
        for transform_stamped in msg.transforms:
            # 可选：覆盖坐标系（如果需要固定父/子坐标系）
            # transform_stamped.header.frame_id = 'odom'    # 强制父坐标系
            # transform_stamped.child_frame_id = 'base_link'# 强制子坐标系
            
            # 发布到TF树
            self.tf_broadcaster.sendTransform(transform_stamped)
            self.get_logger().debug(
                f"发布TF变换: {transform_stamped.header.frame_id} → {transform_stamped.child_frame_id}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = TfOdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被手动终止")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()