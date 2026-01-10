import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_path = FindPackageShare('slam_rtabmap')
    
    return LaunchDescription([

        DeclareLaunchArgument(
            'config_path',
            default_value=PathJoinSubstitution([pkg_path, 'config', 'rtabmap_localization.yaml']),
            description='Path to localization config file'
        ),
        
        DeclareLaunchArgument(
            'map_path',
            default_value='~/maps/rtabmap.db',
            description='Path to the map database'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'
        ),
        
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                LaunchConfiguration('config_path'),
                {
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'database_path': LaunchConfiguration('map_path'),
                }
            ],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('scan', '/scan'),
                ('odom', '/mecanum_drive_controller/odometry'),
            ],
        ),
    ])