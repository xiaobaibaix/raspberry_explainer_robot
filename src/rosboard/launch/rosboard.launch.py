from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 定义端口参数
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8888',
        description='ROSboard listening port'
    )

    # 定义 rosboard 节点
    rosboard_node = Node(
        package='rosboard',
        executable='rosboard_node',
        name='rosboard',
        output='screen',
        parameters=[{'port': LaunchConfiguration('port')}]
    )

    # 返回 launch 描述
    return LaunchDescription([
        port_arg,
        rosboard_node
    ])