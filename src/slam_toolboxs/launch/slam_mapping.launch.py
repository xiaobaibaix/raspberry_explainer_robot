from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    # 获取包路径
    pkg_share = get_package_share_directory('my_robot_bringup')
    
    # 配置文件路径
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox_config.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'slam.rviz')
    
    # ===== Launch参数 =====
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='是否使用仿真时间'
    )
    
    # ===== 1. 启动雷达 =====
    
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'lidar.launch.py')
        )
    )
    
    # ===== 2. 启动里程计 =====
    
    odom_node = Node(
        package='my_robot_bringup',
        executable='odom_node',
        name='odom_node',
        output='screen'
    )
    
    # ===== 3. SLAM节点 =====
    
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }
        ]
    )
    
    # ===== 4. RViz可视化 =====
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        lidar_launch,
        odom_node,
        slam_node,
        rviz_node
    ])