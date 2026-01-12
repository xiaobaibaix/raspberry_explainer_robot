#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped, PoseArray
from robot_msgs.msg import VoiceState, AppRouteUnit
from std_msgs.msg import Bool
from robot_msgs.msg import AppSetRoute
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from enum import Enum
import math
from std_msgs.msg import Int8
import threading
from typing import List, Dict, Tuple, Optional
import time


class NavigatorStateEnum(Enum):
    Running = 0
    Stop = 1
    Pause = 2


class NavigatorApp(Node):
    def __init__(self):
        super().__init__('nav2_through_pose_navigator_app')
        self.navigator = BasicNavigator()

        # 设置初始位置
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.z = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(initial_pose)

        # 等待nav2激活
        self.navigator.waitUntilNav2Active()

        # 订阅器
        self.create_subscription(
            AppSetRoute,
            '~/set_through_poses',
            self.goal_poses_callback, 10
        )

        self.create_subscription(
            Int8,
            '~/set_navigation_state',
            self.set_navigation_state_callback, 10
        )

        self.create_subscription(
            PoseArray,
            'chassis_node/voice_end',
            self.voice_end_callback, 10
        )

        # 发布器
        self.voice_pub = self.create_publisher(
            VoiceState,
            'chassis_node/set_voice_state',
            10
        )

        # 状态变量
        self.nav2_state = NavigatorStateEnum.Stop
        self.is_navigating = False  # 是否正在导航到一个目标点
        self.cur_pose_index = 0  # 当前导航起点索引
        self.is_played = False  # 当前路径语音是否已播放
        self.nav_thread_active = False  # 导航线程是否活跃
        self.nav_thread_stop_flag = False  # 导航线程停止标志
        self.voice_state_lock = threading.Lock()  # 语音状态锁

        # 数据存储
        self.voices_map: Dict[int, Tuple[int, float, int]] = {}  # 语音映射
        self.goal_poses: List[AppRouteUnit] = []  # 导航目标点列表
        self.current_voice_id = -1  # 当前播放的语音ID

        # 线程控制
        self.nav_thread: Optional[threading.Thread] = None
        self.nav_condition = threading.Condition()

        self.get_logger().info('Navigator app has been started.')

    def voice_end_callback(self, msg: PoseArray) -> None:
        """语音播放结束回调"""
        with self.voice_state_lock:
            self.is_played = True
            self.get_logger().info(f'Voice playback completed for voice ID')

    def set_navigation_state_callback(self, msg: Int8) -> None:
        """设置导航状态回调"""
        try:
            new_state = NavigatorStateEnum(msg.data)
        except ValueError:
            self.get_logger().warn(f'Invalid navigation state: {msg.data}')
            return

        with self.nav_condition:
            old_state = self.nav2_state
            self.nav2_state = new_state
            
            if new_state == NavigatorStateEnum.Stop:
                # 停止导航
                if self.is_navigating:
                    self.navigator.cancelTask()
                
                # 停止导航线程
                self.nav_thread_stop_flag = True
                self.nav_condition.notify_all()
                
                self.get_logger().info('Navigator is stopped.')
                
            elif new_state == NavigatorStateEnum.Pause:
                if self.is_navigating:
                    self.navigator.pauseTask()
                self.get_logger().info('Navigator is paused.')
                
            elif new_state == NavigatorStateEnum.Running:
                if not self.nav_thread_active and self.goal_poses:
                    # 启动导航线程
                    self.nav_thread_stop_flag = False
                    self.nav_thread = threading.Thread(target=self.navigator_thread_func, daemon=True)
                    self.nav_thread.start()
                    self.get_logger().info('Navigator is started.')
                elif self.is_navigating and old_state == NavigatorStateEnum.Pause:
                    self.navigator.resumeTask()
                    self.get_logger().info('Navigator is resumed.')

    def goal_poses_callback(self, msg: AppSetRoute) -> None:
        """目标点回调"""
        with self.nav_condition:
            # 停止当前导航
            if self.nav2_state == NavigatorStateEnum.Running and self.is_navigating:
                self.navigator.cancelTask()
            
            # 更新数据
            self.goal_poses = msg.poses
            self.voices_map = {}
            for v in msg.map:
                self.voices_map[v.from_idex] = (v.to_idex, v.play_percent, v.play_id)
            
            # 重置状态
            self.cur_pose_index = 0
            
            # 如果当前是运行状态且有目标点，启动导航线程
            if self.nav2_state == NavigatorStateEnum.Running and len(self.goal_poses) > 0:
                if not self.nav_thread_active:
                    self.nav_thread_stop_flag = False
                    self.nav_thread = threading.Thread(target=self.navigator_thread_func, daemon=True)
                    self.nav_thread.start()
            
            self.get_logger().info(f'Received {len(self.goal_poses)} goal poses.')

    def calculate_distance(self, pose1: PoseStamped, pose2: PoseStamped) -> float:
        """计算两个点之间的欧氏距离"""
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx*dx + dy*dy)

    def wait_for_delay_or_stop(self, delay_seconds: float) -> bool:
        """等待指定时间，但如果收到停止信号则提前返回"""
        if delay_seconds <= 0:
            return True
            
        start_time = time.time()
        with self.nav_condition:
            while (time.time() - start_time) < delay_seconds:
                if self.nav_thread_stop_flag:
                    return False
                    
                remaining = delay_seconds - (time.time() - start_time)
                if remaining > 0:
                    self.nav_condition.wait(timeout=min(0.5, remaining))
            return True

    def publish_voice_command(self, voice_id: int, state: int) -> None:
        """发布语音命令，不等待完成"""
        voice_msg = VoiceState()
        voice_msg.id = voice_id
        voice_msg.state = state  # 使用传入的状态
        self.voice_pub.publish(voice_msg)
        self.get_logger().info(f'Published voice command {voice_id}')
        
        with self.voice_state_lock:
            self.is_played = False
            self.current_voice_id = voice_id

    def navigator_thread_func(self) -> None:
        """导航线程主函数"""
        with self.nav_condition:
            if self.nav_thread_active:
                self.get_logger().warn('Navigation thread is already running!')
                return
            self.nav_thread_active = True
            self.get_logger().info('Navigation thread started.')

        try:
            for i in range(len(self.goal_poses)):
                with self.nav_condition:
                    if self.nav_thread_stop_flag:
                        break
                
                # 检查停止标志
                if self.nav_thread_stop_flag:
                    break
                
                # 获取当前目标点
                current_goal = self.goal_poses[i]
                
                # 延迟执行
                if not self.wait_for_delay_or_stop(current_goal.start_time):
                    break
                
                with self.nav_condition:
                    if self.nav_thread_stop_flag or self.nav2_state != NavigatorStateEnum.Running:
                        break
                
                # 导航到目标点
                self.get_logger().info(f'Navigating to pose {i}')
                
                with self.nav_condition:
                    self.is_navigating = True
                    self.cur_pose_index = i
                
                # 创建导航任务
                go_to_pose_task = self.navigator.goToPose(current_goal.pose)
                
                # 获取语音信息
                voice_info = self.voices_map.get(i, (None, None, None))
                target_index, play_percent, voice_id = voice_info
                
                # 计算路径长度（如果有语音）
                distance = 0.0
                if target_index is not None and target_index < len(self.goal_poses):
                    distance = self.calculate_distance(
                        current_goal.pose,
                        self.goal_poses[target_index].pose
                    )
                    self.get_logger().info(
                        f'Will play voice {voice_id} at {play_percent*100}% of distance {distance:.2f}m'
                    )
                
                # 处理语音播放逻辑
                voice_played = False
                if voice_id is not None and play_percent <= 0.01:
                    # 立即播放语音
                    self.publish_voice_command(voice_id, state=1)
                    voice_played = True
                
                # 监控导航任务
                while not self.navigator.isTaskComplete(task=go_to_pose_task):
                    with self.nav_condition:
                        if self.nav_thread_stop_flag or self.nav2_state == NavigatorStateEnum.Stop:
                            self.navigator.cancelTask()
                            break
                        elif self.nav2_state == NavigatorStateEnum.Pause:
                            self.nav_condition.wait()
                            continue
                    
                    # 获取反馈
                    feedback = self.navigator.getFeedback(task=go_to_pose_task)
                    
                    # 处理语音播放
                    if (not voice_played and voice_id is not None and target_index is not None and
                        distance > 0 and feedback.distance_remaining > 0):
                        
                        current_percent = 1.0 - (feedback.distance_remaining / distance)
                        if abs(current_percent - play_percent) < 0.05:
                            self.publish_voice_command(voice_id, state=1)
                            voice_played = True
                    
                    # 定期记录状态
                    if feedback.navigation_time.sec % 5 == 0:  # 每5秒记录一次
                        self.get_logger().info(
                            f'ETA: {feedback.estimated_time_remaining.sec:.0f} seconds, '
                            f'Remaining distance: {feedback.distance_remaining:.2f}m'
                        )
                    
                    # 检查超时
                    if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                        self.get_logger().warn('Navigation timed out after 10 minutes')
                        self.navigator.cancelTask()
                        break
                
                # 获取导航结果
                with self.nav_condition:
                    self.is_navigating = False
                    
                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f'Pose {i} succeeded!')
                elif result == TaskResult.CANCELED:
                    self.get_logger().info(f'Pose {i} was canceled!')
                    break
                elif result == TaskResult.FAILED:
                    error_code, error_msg = self.navigator.getTaskError()
                    self.get_logger().error(f'Pose {i} failed! {error_code}:{error_msg}')
                    break
                    
        except Exception as e:
            self.get_logger().error(f'Error in navigation thread: {str(e)}')
            
        finally:
            with self.nav_condition:
                self.is_navigating = False
                self.nav_thread_active = False
                self.get_logger().info('Navigation thread finished.')

    def destroy_node(self) -> None:
        """节点销毁时的清理工作"""
        with self.nav_condition:
            self.nav_thread_stop_flag = True
            self.nav_condition.notify_all()
        
        if self.nav_thread is not None and self.nav_thread.is_alive():
            self.nav_thread.join(timeout=2.0)
        
        self.navigator.lifecycleShutdown()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    
    try:
        navigator_app = NavigatorApp()
        rclpy.spin(navigator_app)
        
    except KeyboardInterrupt:
        navigator_app.get_logger().info('Shutdown by keyboard interrupt')
    except Exception as e:
        navigator_app.get_logger().error(f'Program error: {type(e).__name__}: {e}')
    finally:
        if 'navigator_app' in locals():
            navigator_app.destroy_node()
        
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()