#!/usr/bin/env python3
from geometry_msgs.msg import Pose,PoseStamped,PoseWithCovarianceStamped
from robot_msgs.msg import VoiceState, AppRouteUnit, AppSetRoute, AppVoicePlay
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from enum import Enum
import math
from std_msgs.msg import Int8
from typing import List, Dict, Tuple, Optional, Set, Deque
import time
from dataclasses import dataclass
from collections import defaultdict, deque
import bisect


@dataclass
class VoiceTrigger:
    voice_id: int
    play_percent: float



class NavigatorStateEnum(Enum):
    Running = 0
    Stop = 1
    Pause = 2

class NavigationStep(Enum):
    IDLE = 0
    WAITING_FOR_START = 1
    NAVIGATING = 2
    COMPLETE = 3
    FAILED = 4

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
        self.get_logger().info('Waiting for Nav2 to activate...')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active!')
        # 订阅器
        self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback, 10
        )
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
            VoiceState,
            'chassis_node/voice_end',
            self.voice_end_callback, 10
        )

        # 发布器
        self.voice_pub = self.create_publisher(
            VoiceState,
            'chassis_node/set_voice_state',
            10
        )
        self.state_pub = self.create_publisher(
            Int8,
            '~/ret_nav2_finish_state',
            10
        )

        # 状态变量
        self.nav2_state = NavigatorStateEnum.Stop
        self.navigation_step = NavigationStep.IDLE
        self.current_pose_index = 0     # 当前导航目标点索引
        self.current_segment_total_distance =0.0  # 当前段落总距离
        self.current_segment_voices: Deque[VoiceTrigger] = deque()  # 当前段落的语音触发队列
        self.task_start_time= None  # 任务开始时间
        self.init_pose: PoseStamped = initial_pose # 导航最开始的位置
        self.last_log_time= None  # 上次日志记录时间
        self.last_time_navigation_timer_callback= None  # 上次navigation_timer_callback时间
        self.cur_play_voice_id = 0
        self.cur_play_voice_id_is_end = True
        # 数据存储
        # 语音映射字典: 起点索引 -> 语音列表[(百分比, 语音ID)]
        self.voices_map: Dict[int, List[Tuple[float, int]]] = defaultdict(list)
        self.goal_poses: List[AppRouteUnit] = []  # 导航目标点列表

        # 定时器
        self.timer = self.create_timer(0.2, self.navigation_timer_callback)  # 200ms定时器

        self.get_logger().info('Navigator app has been started.')

    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped) -> None:
        """AMCL位置回调"""
        self.init_pose = PoseStamped()
        self.init_pose.header = msg.header
        self.init_pose.pose = msg.pose.pose
        
        self.get_logger().info(
            f'AMCL pose updated: Position ({msg.pose.pose.position.x}, '
            f'{msg.pose.pose.position.y})'
        )

    def voice_end_callback(self, msg: VoiceState) -> None:
        """语音播放结束回调"""
        self.get_logger().info(f'Voice playback completed for voice ID {msg.id}')
        self.cur_play_voice_id_is_end = True

    def set_navigation_state_callback(self, msg: Int8) -> None:
        """设置导航状态回调"""
        try:
            new_state = NavigatorStateEnum(msg.data)
        except ValueError:
            self.get_logger().warn(f'Invalid navigation state: {msg.data}')
            return
        # 状态转换处理，打印日志
        old_state = self.nav2_state
        self.nav2_state = new_state
        self.get_logger().info(f'Navigation state changed from {old_state.name} to {new_state.name}')
        if new_state == NavigatorStateEnum.Stop:
            # 停止导航
            if self.navigation_step == NavigationStep.NAVIGATING:
                self.navigator.cancelTask()
                self.get_logger().info('Navigation task cancelled.')
            
            # 重置状态
            self.navigation_step = NavigationStep.IDLE
            self.current_pose_index = 0
            self.current_segment_total_distance=0.0
            self.current_segment_voices.clear()
            self.task_start_time = None
            self.cur_play_voice_id = 0
            self.cur_play_voice_id_is_end = True

            if self.cur_play_voice_id_is_end == False:
                # 停止当前语音播放
                self.publish_voice_command(self.cur_play_voice_id, 0)
                self.cur_play_voice_id = 0
                self.cur_play_voice_id_is_end = True
                self.get_logger().info('Current voice playback cancelled.')
            
            self.get_logger().info('Navigator is stopped.')
            
        elif new_state == NavigatorStateEnum.Pause:
            if self.navigation_step == NavigationStep.NAVIGATING:
                self.navigator.cancelTask()
                self.get_logger().info('Navigator is NAVIGATING paused.')
            elif self.navigation_step == NavigationStep.WAITING_FOR_START:
                self.get_logger().info('Navigator is WAITING_FOR_START while waiting to start.')
            
            if self.cur_play_voice_id_is_end == False:
                # 停止当前语音播放
                self.publish_voice_command(self.cur_play_voice_id, 0)
                self.cur_play_voice_id = 0
                self.cur_play_voice_id_is_end = True
                self.get_logger().info('Current voice playback cancelled.')

        elif new_state == NavigatorStateEnum.Running:
            if old_state == NavigatorStateEnum.Stop:
                # 从停止状态开始
                if self.goal_poses:
                    self.task_start_time = self.get_clock().now()
                    self.current_pose_index = 0
                    self.navigation_step = NavigationStep.WAITING_FOR_START
                    self.get_logger().info('Starting navigation from stop state.')
                else:
                    self.get_logger().warn('Cannot start navigation: no goal poses set.')
                    self.nav2_state = NavigatorStateEnum.Stop
                    
            elif old_state == NavigatorStateEnum.Pause:
                if self.navigation_step == NavigationStep.NAVIGATING:
                    self.get_logger().info('Navigator is resumed.')
                    # 上一次已经进入导航状态，直接开始导航
                    current_goal = self.goal_poses[self.current_pose_index]
                    # 可能需要触发起点语音
                    self.get_logger().info(f'Resuming navigation to pose {self.current_pose_index}')
                    if self.navigator.goToPose(current_goal.pose):
                        self.get_logger().info(f'Navigation to pose {self.current_pose_index} resumed successfully.')
                    else:
                        self.get_logger().error(f'Failed to resume navigation to pose {self.current_pose_index}')
                        self.navigation_step = NavigationStep.FAILED

                elif self.navigation_step == NavigationStep.WAITING_FOR_START:
                    self.get_logger().warn('Cannot resume: not currently navigating.')
                    # 重新开始延迟，延迟到需求以后再开始导航
                    self.task_start_time = self.get_clock().now()

    def goal_poses_callback(self, msg: AppSetRoute) -> None:
        """目标点回调"""
        # 停止当前导航
        if self.nav2_state == NavigatorStateEnum.Running and self.navigation_step == NavigationStep.NAVIGATING:
            self.navigator.cancelTask()
            self.get_logger().info('Current navigation cancelled due to new goal poses.')
            self.nav2_state == NavigatorStateEnum.Stop
        
        # 清空旧的语音映射
        self.voices_map.clear()
        
        # 更新目标点
        self.goal_poses = msg.poses
        
        # 更新语音映射
        for v in msg.map:
            # 语音映射: (百分比, 语音ID, 目标点索引)
            self.voices_map[v.from_idex].append((v.play_percent, v.play_id))
        
        # 重置状态
        self.current_pose_index = 0
        self.current_segment_total_distance=0.0
        self.navigation_step = NavigationStep.IDLE
        self.current_segment_voices.clear()
        self.task_start_time = None
        self.cur_play_voice_id = 0
        self.cur_play_voice_id_is_end = True
        
        # 打印接收了多少个目标点以及对应的详细信息
        self.get_logger().info(f'Received {len(self.goal_poses)} goal poses.')
        for i, pose_unit in enumerate(self.goal_poses):
            self.get_logger().info(
                f'Pose {i}: Start time {pose_unit.start_time}s, '
                f'Position ({pose_unit.pose.pose.position.x}, {pose_unit.pose.pose.position.y})'
            )
        
        # 打印语音映射信息
        total_voice_triggers = sum(len(triggers) for triggers in self.voices_map.values())
        self.get_logger().info(f'Received {total_voice_triggers} voice triggers.')
        
        for from_idx, triggers in self.voices_map.items():
            for percent, voice_id in triggers:
                play_type = "start" if percent == 0.0 else "end" if percent >= 0.999 else f"{percent*100:.1f}%"
                self.get_logger().info(
                    f'Voice trigger: from {from_idx}, '
                    f'play ID {voice_id} at {play_type}'
                )

    def calculate_distance(self, pose1: Pose, pose2: Pose) -> float:
        """计算两个点之间的欧氏距离"""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        return math.sqrt(dx*dx + dy*dy)

    def publish_voice_command(self, voice_id: int, state: int) -> None:
        """发布语音命令"""
        voice_msg = VoiceState()
        voice_msg.id = voice_id
        voice_msg.state = state
        self.voice_pub.publish(voice_msg)
        self.get_logger().info(f'Published voice command {voice_id} with state {state}')

    def check_and_trigger_voices(self, distance_remaining) -> None:
        """检查并触发多个语音播放"""
        #队列为空或者总距离为0，直接返回
        if not self.current_segment_voices or math.isclose(self.current_segment_total_distance, 0.0):
            return
            
        # 计算当前完成百分比
        current_percent = 1.0 - (distance_remaining / self.current_segment_total_distance)
        
        # 检查队列中的语音触发点
        while self.current_segment_voices:
            voice_trigger = self.current_segment_voices[0]  # 查看队列第一个
            
            # 如果当前百分比大于触发百分比
            if current_percent > voice_trigger.play_percent or math.isclose(current_percent, voice_trigger.play_percent):
                # 触发语音播放
                self.publish_voice_command(voice_trigger.voice_id, 1)
                self.cur_play_voice_id = voice_trigger.voice_id
                self.cur_play_voice_id_is_end = False
                self.get_logger().info(f'Triggered voice {voice_trigger.voice_id} at {current_percent*100:.1f}% (trigger at {voice_trigger.play_percent*100}%)')
                # 从队列中移除
                self.current_segment_voices.popleft()
            else:
                break  # 队列第一个未到触发点，退出循环

    def setup_segment_voices(self, to_index: int) -> None:
        """设置当前段落的语音触发队列"""
        self.current_segment_voices.clear()
        
        # 获取当前段落的语音触发点
        if to_index in self.voices_map:
            voice_triggers = []
            for play_percent, voice_id in self.voices_map[to_index]:
                # 只添加目标点是当前目标点的语音
                voice_trigger = VoiceTrigger(
                    voice_id=voice_id,
                    play_percent=play_percent,
                )
                voice_triggers.append(voice_trigger)
            
            # 按百分比排序
            voice_triggers.sort(key=lambda x: x.play_percent)
            
            # 添加到队列
            for trigger in voice_triggers:
                self.current_segment_voices.append(trigger)
            
            self.get_logger().info(f'Added {len(voice_triggers)} voice triggers for segment {to_index}')
        
        # 计算段落总距离 
        if(to_index ==0):
            current_segment_start_pose = self.init_pose.pose
            current_segment_end_pose = self.goal_poses[to_index].pose.pose
            self.current_segment_total_distance = self.calculate_distance(
                current_segment_start_pose,
                current_segment_end_pose
            )
            self.get_logger().info(f'Segment 0: total distance = {self.current_segment_total_distance:.2f}m')
        else:
            if to_index < len(self.goal_poses):
                current_segment_start_pose = self.goal_poses[to_index-1].pose.pose
                current_segment_end_pose = self.goal_poses[to_index].pose.pose
                self.current_segment_total_distance = self.calculate_distance(
                    current_segment_start_pose,
                    current_segment_end_pose
                )
                self.get_logger().info(f'Segment {to_index-1}->{to_index}: total distance = {self.current_segment_total_distance:.2f}m')
            else:
                self.current_segment_total_distance = 0.0
                self.get_logger().warn(f'Invalid segment indices: {to_index}')


    def navigation_timer_callback(self) -> None:
        """定时器回调，处理导航状态机"""


        # 如果不在运行状态，不处理
        if self.nav2_state != NavigatorStateEnum.Running:
            return
            
        # 如果没有目标点，不处理
        if not self.goal_poses:
            return
            
        # 根据当前状态处理
        if self.navigation_step == NavigationStep.IDLE:
            # 空闲状态，等待开始
            pass
            
        elif self.navigation_step == NavigationStep.WAITING_FOR_START:
                
            current_goal = self.goal_poses[self.current_pose_index]
            elapsed_time = (self.get_clock().now() - self.task_start_time).nanoseconds / 1e9
            self.get_logger().info('Waiting... Elapsed time: {:.1f}s, Goal start time: {:.1f}s'.format(elapsed_time, current_goal.start_time))
            # 超过等待时间才开始导航
            if elapsed_time >= current_goal.start_time:
                # 开始导航到当前目标点
                self.get_logger().info(f'Starting navigation to pose {self.current_pose_index}')
                self.get_logger().info(
                    f'Pose details: Position ({current_goal.pose.pose.position.x}, '
                    f'{current_goal.pose.pose.position.y})'
                )
                # 设置当前段落的语音触发队列
                self.setup_segment_voices(self.current_pose_index)
                
                # 触发起点语音
                self.check_and_trigger_voices(distance_remaining=self.current_segment_total_distance)
                
                # 开始导航
                if self.navigator.goToPose(current_goal.pose):
                    self.navigation_step = NavigationStep.NAVIGATING
                    self.get_logger().info(f'Navigation to pose {self.current_pose_index} started successfully.')
                else:
                    self.get_logger().error(f'Failed to start navigation to pose {self.current_pose_index}')
                    self.navigation_step = NavigationStep.FAILED
                    
        elif self.navigation_step == NavigationStep.NAVIGATING:
            # 检查导航状态
            self.last_time_navigation_timer_callback = self.get_clock().now()
            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                
                if result == TaskResult.SUCCEEDED:
                    # 触发终点语音（百分比为1.0）
                    self.check_and_trigger_voices(0)
                    self.get_logger().info(f'Pose {self.current_pose_index} succeeded!')
                    self.current_pose_index += 1
                    
                    if self.current_pose_index >= len(self.goal_poses):
                        self.navigation_step = NavigationStep.COMPLETE
                        self.get_logger().info('All poses completed!')
                    else:
                        self.navigation_step = NavigationStep.WAITING_FOR_START
                        self.get_logger().info(f'Moving to next pose {self.current_pose_index}')
                        
                elif result == TaskResult.CANCELED:
                    self.get_logger().info(f'Pose {self.current_pose_index} was canceled!')
                    self.navigation_step = NavigationStep.IDLE
                    self.nav2_state = NavigatorStateEnum.Stop
                    
                elif result == TaskResult.FAILED:
                    self.get_logger().error(f'Pose {self.current_pose_index} failed!')
                    self.navigation_step = NavigationStep.FAILED
                    self.nav2_state = NavigatorStateEnum.Stop
                else:
                    self.get_logger().error('Navigation has an invalid return status!')
                    self.navigation_step = NavigationStep.FAILED
                    self.nav2_state = NavigatorStateEnum.Stop
            else:
                # 导航进行中，获取反馈
                try:
                    # self.last_time_navigation_timer_callback = self.get_clock().now()
                    feedback = self.navigator.getFeedback()
                    if feedback:
                        # 检查并触发语音
                        self.check_and_trigger_voices(feedback.distance_remaining)
                        
                        # 定期记录状态（每5秒）
                        current_time = self.get_clock().now()
                        if self.last_log_time is None or (current_time - self.last_log_time).nanoseconds / 1e9 >= 5.0:
                            self.last_log_time = current_time
                            self.get_logger().info(
                                f'ETA: {feedback.estimated_time_remaining.sec:.0f} seconds, '
                                f'Remaining distance: {feedback.distance_remaining:.2f}m'
                            )
                        
                        # 检查超时
                        if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                            self.get_logger().warn('Navigation timed out after 10 minutes')
                            self.navigator.cancelTask()
                            self.navigation_step = NavigationStep.FAILED
                            self.nav2_state = NavigatorStateEnum.Stop
                            
                except Exception as e:
                    # 获取反馈可能会失败，这通常是正常的
                    pass
                    
        elif self.navigation_step == NavigationStep.COMPLETE:
            # 所有目标点完成
            self.get_logger().info('Navigation complete!')
            self.navigation_step = NavigationStep.IDLE
            self.nav2_state = NavigatorStateEnum.Stop
            self.state_pub.publish(Int8(data=1))  # 发布完成状态
            
        elif self.navigation_step == NavigationStep.FAILED:
            # 导航失败
            self.get_logger().error('Navigation failed!')
            self.navigation_step = NavigationStep.IDLE
            self.nav2_state = NavigatorStateEnum.Stop
            self.state_pub.publish(Int8(data=2))  # 发布失败状态

        # if self.last_time_navigation_timer_callback:
        #     delta_navigation_timer_callback=self.get_clock().now()-self.last_time_navigation_timer_callback
        #     self.get_logger().info(f'navigation_timer_callback execution time: {delta_navigation_timer_callback.nanoseconds / 1e6:.2f} ms')

    def destroy_node(self) -> None:
        """节点销毁时的清理工作"""
        # 停止所有任务
        if self.navigation_step == NavigationStep.NAVIGATING:
            self.navigator.cancelTask()
            
        # 关闭导航器
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