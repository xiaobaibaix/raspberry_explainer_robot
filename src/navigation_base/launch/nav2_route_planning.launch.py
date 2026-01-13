import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    controller_yaml = os.path.join(get_package_share_directory('navigation'), 'config', 'controllers.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('navigation'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('navigation'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('navigation'), 'config', 'recovery.yaml')
    smoother_yaml = os.path.join(get_package_share_directory('navigation'), 'config', 'smoother.yaml')
    waypoint_yaml = os.path.join(get_package_share_directory('navigation'), 'config', 'waypoint.yaml') 
    nav2_params = os.path.join(get_package_share_directory('navigation'), 'config', 'nav2_params_my.yaml')

    return LaunchDescription([     
        # 速度加时间戳节点
        Node(
            package='navigation',
            executable='twist_stamper',
            name='twist_stamper',
            output='screen',
        ),
        # 控制器节点
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
        ),

        # 规划器节点
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params]),
            
        # 行为服务节点
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            parameters=[nav2_params],
            output='screen'),

        # BT导航器
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params]),
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params]
        ),
        # 平滑器节点
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_params],
        ),

        # 速度平滑器
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params],
        ),

        # 生命周期管理
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': [
                            'planner_server',
                            'controller_server',
                            'behavior_server',
                            'bt_navigator',
                            'smoother_server',
                            'velocity_smoother'
                        ]}],
        ),
    ])