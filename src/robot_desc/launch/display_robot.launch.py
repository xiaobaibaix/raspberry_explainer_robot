from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os
import subprocess
import robot_desc.urdf_from_xacro as urdf_util

from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    packagepath = get_package_share_directory("robot_desc")
    robot_desc = urdf_util.get_urdf_from_xacro('/'.join([packagepath, 'urdf', 'four_wheel_chassis.xacro']))

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_desc',
            default_value='false',
            description='Robot description in URDF format',
        ),
        # 将URDF模型加载到参数服务器
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        
        # 启动joint_state_publisher（可选，用于手动调整关节）
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen'
        ),
        # 启动joint_state_publisher_gui（可选，用于图形界面调整关节）
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_gui', default='false'))
        ),

        # 启动RViz进行可视化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(packagepath, 'config', 'robot_car.rviz')]
        ),

    ])