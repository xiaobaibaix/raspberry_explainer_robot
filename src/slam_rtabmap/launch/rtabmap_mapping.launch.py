import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径
    pkg_path = get_package_share_directory('slam_rtabmap')
    
    config_path = DeclareLaunchArgument(
        'config_path',
        default_value=PathJoinSubstitution([pkg_path, 'config', 'rtabmap_mapping.yaml']),
        description='Path to RTAB-Map configuration file'
    )
    
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # RTAB-Map节点
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            LaunchConfiguration('config_path'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['--delete_db_on_start'],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/depth/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('scan', '/scan'),
            ('odom', '/mecanum_drive_controller/odometry'),
        ],
    )
    
    # 可选的深度图转激光扫描节点
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -3.14159,  # -M_PI
            'angle_max': 3.14159,   # M_PI
            'angle_increment': 0.0087,  # 0.5 degree
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 4.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        remappings=[
            ('cloud_in', '/camera/depth/points'),
            ('scan', '/scan_from_depth'),
        ],
    )
    
    return LaunchDescription([
        # 声明参数
        config_path,
        use_sim_time,
        
        # 启动节点
        rtabmap_node,
        pointcloud_to_laserscan,  # 如果需要深度转激光，取消注释
    ])