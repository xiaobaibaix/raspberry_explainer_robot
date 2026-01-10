from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch_ros.actions import Node as LaunchNode
import subprocess
import os

def generate_launch_description():

    robot_path = get_package_share_directory("robot_car_mecanum")
    nav2_path = get_package_share_directory("navigation")

    ld= LaunchDescription()

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    robot_path,
                    'launch',
                    'robot_driver.launch.py'
                ),
            ])
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    robot_path,
                    'launch',
                    'robot_ros2_control.launch.py'
                ),
            ]),
            launch_arguments={
                'robot_desc': 'robot_car_voice.xacro',
            }.items()
        )
    )

    # nav2_local=IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             nav2_path,
    #             'launch',
    #             'nav2_local_localization.launch.py'
    #         ),
    #     ])  
    # )
    # ld.add_action(TimerAction(
    #     period=5.0,
    #     actions=[nav2_local],
    # ))
    # nav2_rout=IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             nav2_path,
    #             'launch',
    #             'nav2_route_planning.launch.py'
    #         ),
    #     ])  
    # )
    # ld.add_action(TimerAction(
    #     period=10.0,
    #     actions=[nav2_rout],
    # ))
    return ld