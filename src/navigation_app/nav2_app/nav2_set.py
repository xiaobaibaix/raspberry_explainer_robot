#!/usr/bin/env python3
"""
交互式导航巡航启动器
功能：加载YAML配置文件，按回车自动发布导航相关话题
"""

import os
import sys
import yaml
import time
import rclpy
from rclpy.node import Node
from robot_msgs.msg import AppSetRoute, AppRouteUnit, AppVoicePlay
from std_msgs.msg import Int8
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Header

class InteractiveNavigationStarter(Node):
    def __init__(self, yaml_path, wait_time=2.0):
        """
        初始化交互式导航启动器
        
        参数:
        yaml_path: YAML配置文件路径
        wait_time: 发布路径点后等待的时间（秒）
        """
        super().__init__('interactive_navigation_starter')
        
        # 发布者
        self.route_publisher = self.create_publisher(
            AppSetRoute, 
            '/nav2_through_pose_navigator_app/set_through_poses', 
            10
        )
        
        self.nav_state_publisher = self.create_publisher(
            Int8,
            '/nav2_through_pose_navigator_app/set_navigation_state',
            10
        )
        
        self.yaml_path = yaml_path
        self.wait_time = wait_time
        self.current_route = None
        
        self.get_logger().info(f'交互式导航启动器已启动')
        self.get_logger().info(f'配置文件: {yaml_path}')
        self.get_logger().info(f'等待时间: {wait_time}秒')
    
    def load_route(self):
        """加载YAML文件"""
        try:
            if not os.path.exists(self.yaml_path):
                self.get_logger().error(f"文件不存在: {self.yaml_path}")
                return False
            
            with open(self.yaml_path, 'r') as f:
                yaml_data = yaml.safe_load(f)
            
            self.current_route = self.create_route_from_yaml(yaml_data)
            self.get_logger().info(f'成功加载配置文件，包含 {len(self.current_route.poses)} 个路径点')
            return True
            
        except yaml.YAMLError as e:
            self.get_logger().error(f"YAML解析错误: {e}")
        except Exception as e:
            self.get_logger().error(f"加载失败: {e}")
        
        return False
    
    def create_route_from_yaml(self, yaml_data):
        """从YAML数据创建路径点消息"""
        route_msg = AppSetRoute()
        
        # 解析poses
        poses_data = yaml_data.get('poses', [])
        for pose_data in poses_data:
            try:
                route_unit = AppRouteUnit()
                route_unit.start_time = int(pose_data.get('start_time', 0))
                
                # 获取路径点信息
                pose_info = pose_data.get('pose', {})
                header_info = pose_info.get('header', {})
                pose_val = pose_info.get('pose', {})
                position_info = pose_val.get('position', {})
                orientation_info = pose_val.get('orientation', {})
                
                # 创建PoseStamped
                pose_stamped = PoseStamped()
                
                # 设置header
                pose_stamped.header = Header(
                    stamp=self.get_clock().now().to_msg(),
                    frame_id=header_info.get('frame_id', 'map')
                )
                
                # 设置位置
                pose_stamped.pose.position = Point(
                    x=float(position_info.get('x', 0.0)),
                    y=float(position_info.get('y', 0.0)),
                    z=float(position_info.get('z', 0.0))
                )
                
                # 设置朝向
                pose_stamped.pose.orientation = Quaternion(
                    x=float(orientation_info.get('x', 0.0)),
                    y=float(orientation_info.get('y', 0.0)),
                    z=float(orientation_info.get('z', 0.0)),
                    w=float(orientation_info.get('w', 1.0))
                )
                
                route_unit.pose = pose_stamped
                route_msg.poses.append(route_unit)
                
            except Exception as e:
                self.get_logger().warning(f"解析路径点时出错: {e}")
        
        # 解析voice map
        map_data = yaml_data.get('map', [])
        for map_item in map_data:
            try:
                voice_play = AppVoicePlay()
                voice_play.from_idex = int(map_item.get('from_idex', 0))
                voice_play.play_id = int(map_item.get('play_id', 0))
                voice_play.play_percent = float(map_item.get('play_percent', 0.0))
                route_msg.map.append(voice_play)
                
            except Exception as e:
                self.get_logger().warning(f"解析语音映射时出错: {e}")
        
        return route_msg
    
    def publish_once(self):
        """执行一次完整的发布流程"""
        try:
            # 步骤1: 加载YAML文件
            if not self.load_route():
                self.get_logger().error(f'加载配置文件失败: {self.yaml_path}')
                return False
            
            # 步骤2: 发布路径点
            self.get_logger().info('发布路径点配置...')
            self.publish_route()
            
            # 步骤3: 等待指定时间
            self.get_logger().info(f'等待 {self.wait_time} 秒...')
            time.sleep(self.wait_time)
            
            # 步骤4: 发布开始导航命令
            self.get_logger().info('发布导航开始指令...')
            self.start_navigation()
            
            self.get_logger().info('✓ 巡航讲解流程已启动完成')
            return True
            
        except Exception as e:
            self.get_logger().error(f'启动流程失败: {e}')
            return False
    
    def publish_route(self):
        """发布路径点"""
        if self.current_route is None:
            raise ValueError("没有可发布的路径点")
        
        # 更新时间戳
        current_time = self.get_clock().now().to_msg()
        for route_unit in self.current_route.poses:
            route_unit.pose.header.stamp = current_time
        
        self.route_publisher.publish(self.current_route)
        self.get_logger().info(f"已发布路径点，包含 {len(self.current_route.poses)} 个路径点")
    
    def start_navigation(self):
        """开始导航（发布状态1）"""
        msg = Int8()
        msg.data = 0
        self.nav_state_publisher.publish(msg)
        self.get_logger().info("已发布导航开始指令 (状态=1)")
    
    def interactive_loop(self):
        """交互式循环，等待回车输入"""
        print("\n" + "="*60)
        print("交互式导航巡航启动器")
        print("="*60)
        print(f"配置文件: {self.yaml_path}")
        print(f"等待时间: {self.wait_time}秒")
        print("\n使用方法:")
        print("  - 按 [回车] 发送一次导航指令")
        print("  - 输入 'r' 重新加载配置文件")
        print("  - 输入 'w' 修改等待时间")
        print("  - 输入 'i' 显示当前配置信息")
        print("  - 输入 'q' 退出程序")
        print("="*60)
        
        count = 0
        
        while rclpy.ok():
            try:
                # 处理一次ROS回调
                rclpy.spin_once(self, timeout_sec=0.1)
                
                # 获取用户输入
                user_input = input(f"\n[{count}] 按回车发送导航指令，或输入命令: ").strip()
                
                if user_input == 'q' or user_input == 'quit' or user_input == 'exit':
                    print("退出程序")
                    break
                
                elif user_input == 'r' or user_input == 'reload':
                    print("重新加载配置文件...")
                    if self.load_route():
                        print(f"✓ 已重新加载配置文件，包含 {len(self.current_route.poses)} 个路径点")
                    else:
                        print("✗ 重新加载失败")
                
                elif user_input == 'w' or user_input == 'wait':
                    try:
                        new_wait = input(f"当前等待时间: {self.wait_time}秒，请输入新值: ").strip()
                        if new_wait:
                            self.wait_time = float(new_wait)
                            print(f"✓ 等待时间已设置为 {self.wait_time}秒")
                    except ValueError:
                        print("✗ 输入无效，请输入数字")
                
                elif user_input == 'i' or user_input == 'info':
                    self.show_config_info()
                
                elif user_input == '':
                    # 空输入，执行一次发布
                    count += 1
                    print(f"\n[{count}] 开始执行发布流程...")
                    success = self.publish_once()
                    
                    if success:
                        print(f"[{count}] ✓ 发布完成")
                    else:
                        print(f"[{count}] ✗ 发布失败")
                
                else:
                    print("未知命令，可用命令: r(重新加载), w(修改等待时间), i(信息), q(退出)")
                    
            except KeyboardInterrupt:
                print("\n程序被用户中断")
                break
            except EOFError:
                print("\n程序结束")
                break
            except Exception as e:
                self.get_logger().error(f"发生错误: {e}")
    
    def show_config_info(self):
        """显示当前配置信息"""
        if self.current_route is None:
            print("当前没有加载路径点")
            return
        
        print("\n当前配置信息:")
        print(f"  配置文件: {self.yaml_path}")
        print(f"  等待时间: {self.wait_time}秒")
        print(f"  路径点数量: {len(self.current_route.poses)}")
        print(f"  语音映射数量: {len(self.current_route.map)}")
        
        for i, route_unit in enumerate(self.current_route.poses[:3]):  # 只显示前3个
            pos = route_unit.pose.pose.position
            print(f"  路径点{i+1}: 时间={route_unit.start_time}s, 位置=({pos.x:.1f}, {pos.y:.1f})")
        
        if len(self.current_route.poses) > 3:
            print(f"  ... 还有 {len(self.current_route.poses) - 3} 个路径点")

def main():
    """主函数，支持命令行参数"""
    import argparse
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(description='交互式导航巡航启动器')
    parser.add_argument('--yaml', default='set_route.yaml',
                       help='YAML配置文件路径 (默认: set_route.yaml)')
    parser.add_argument('--wait', type=float, default=2.0,
                       help='发布路径点后等待的时间（秒）(默认: 2.0)')
    parser.add_argument('--config-dir', 
                       default=os.path.expanduser('~/workspace/raspberrypi_robot/src/navigation_app/config'),
                       help='配置文件目录 (默认: ~/workspace/raspberrypi_robot/src/navigation_app/config)')
    
    args = parser.parse_args()
    
    # 构建完整的文件路径
    if os.path.isabs(args.yaml):
        yaml_path = args.yaml
    else:
        yaml_path = os.path.join(args.config_dir, args.yaml)
    
    # 检查文件是否存在
    if not os.path.exists(yaml_path):
        print(f"✗ 配置文件不存在: {yaml_path}")
        return 1
    
    # 初始化ROS
    rclpy.init()
    
    try:
        # 创建节点
        node = InteractiveNavigationStarter(yaml_path, args.wait)
        
        # 初始加载配置
        if node.load_route():
            print(f"✓ 初始配置加载成功，包含 {len(node.current_route.poses)} 个路径点")
        else:
            print(f"✗ 初始配置加载失败")
            return 1
        
        # 启动交互式循环
        node.interactive_loop()
        
        # 清理
        node.destroy_node()
        rclpy.shutdown()
        
        print("程序正常退出")
        return 0
            
    except KeyboardInterrupt:
        print("\n程序被用户中断")
        rclpy.shutdown()
        return 130
    except Exception as e:
        print(f"程序运行出错: {e}")
        rclpy.shutdown()
        return 1

if __name__ == '__main__':
    sys.exit(main())