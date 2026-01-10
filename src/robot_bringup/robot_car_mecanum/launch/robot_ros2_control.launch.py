from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
import os
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import GroupAction
from launch.conditions import UnlessCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()


    pkg = get_package_share_directory('robot_car_mecanum')

    ld.add_action(DeclareLaunchArgument(
        'use_ekf',
        default_value='true',
        description='is to use the ekf')
    )

    ld.add_action(DeclareLaunchArgument(
        'start_position_ctrl',
        default_value='false',
        description='open position_tracking_controller')
    )

    ld.add_action(DeclareLaunchArgument(
        'robot_desc',
        default_value='robot_car.xacro',
        description='the robot desc!')
    )

    # 加载机器人描述文件
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'xacro ',
                PathJoinSubstitution([
                    FindPackageShare('robot_car_mecanum'),  # 您的包名
                    'urdf',
                    LaunchConfiguration('robot_desc')
                ])
            ]),
            'use_sim_time': False,
        }],
        output='both'
    )
    ld.add_action(robot_state_publisher)

    # ① 硬件插件 + 控制器管理器（必须同机）
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            os.path.join(pkg, 'config', 'controllers_pid.yaml'),
            # 添加 use_sim_time 参数
            {'use_sim_time': False},
        ],
        output='both',
    )
    ld.add_action(controller_manager)

    # 使用更可靠的事件处理方式
    jspawner = Node(
        package='controller_manager', 
        executable='spawner', 
        arguments=['joint_state_broadcaster'],
        output='both',
        # 添加前缀避免命名空间问题
        prefix='',
    )

    imuspawner = Node(
        package='controller_manager', 
        executable='spawner', 
        arguments=['imu_sensor_broadcaster'],
        output='both',
        prefix='',
    )

    ld.add_action(jspawner) 
    ld.add_action(imuspawner)

    # PID控制器按顺序启动而不是同时启动
    pid_spawner = Node(
        package='controller_manager',
        executable='spawner', 
        arguments=['fl_speed_pid','fr_speed_pid','rl_speed_pid','rr_speed_pid'],
        output='both',
        prefix='',
    )
    
    mecaspawner = Node(
        package='controller_manager',
        executable='spawner', 
        arguments=['mecanum_drive_controller'],
        output='both',
        prefix='',
        # 为麦克纳姆控制器添加额外参数
        parameters=[{'use_sim_time': False}]
    )

    ld.add_action(GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            pkg,  # 注意：pkg需要先通过get_package_share_directory获取路径
                            'launch',
                            'robot_ekf.launch.py'
                        )
                    )
                )
            ],
            condition=IfCondition(LaunchConfiguration('use_ekf'))  # condition写在GroupAction里
        )
    )

    ld.add_action(
        GroupAction(
            actions=[
                Node(
                    package='robot_car_mecanum',
                    executable='tf_odom', 
                    output='both',
                    name='tf_odom',
                )
            ],
            condition=UnlessCondition(LaunchConfiguration('use_ekf'))  # condition写在GroupAction里
        )
    )

    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=jspawner,
            on_exit=[
                pid_spawner
            ]
        )
    ))

    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pid_spawner,
            on_exit=[
                mecaspawner
            ]
        )
    ))

    pid_tack_spawner = Node(
        package='controller_manager',
        executable='spawner', 
        arguments=['mecanum_x_position_pid','mecanum_y_position_pid','mecanum_z_angle_pid'],
        output='both',
        prefix='',
    )

    position_tack_spawner = Node(
        package='controller_manager',
        executable='spawner', 
        arguments=['position_tracking_controller'],
        output='both',
        prefix='',
    )

    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=mecaspawner,
            on_exit=[
                pid_tack_spawner,
            ]
        ),
        condition=IfCondition(LaunchConfiguration('start_position_ctrl'))
    ))

    ld.add_action(RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=pid_tack_spawner,
            on_exit=[
                position_tack_spawner
            ]
        ),
        condition=IfCondition(LaunchConfiguration('start_position_ctrl'))
    ))

    return ld