from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qxlidar_ros2_driver',
            executable='lidar_node',
            name='lidar_node',
            output='screen',
            parameters=[{
                'port': '/dev/ttyAMA0',
                'baud': 115200,
                'model': 'X2M',
                'frame_id': 'laser_frame',
                'inverted': False,
                'scan_size': 1440,   # 0.25°
                'range_min': 0.02,
                'range_max': 12.0,
            }])
    ])