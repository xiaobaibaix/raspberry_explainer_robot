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

    carto_path = get_package_share_directory("slam_cartographer")

    ld= LaunchDescription()

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    carto_path,
                    'launch',
                    'cartographer_base.launch.py'
                ),
            ])
        )
    )

    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    carto_path,
                    'launch',
                    'cartographer_creater.launch.py'
                ),
            ])
        )
    )

    return ld