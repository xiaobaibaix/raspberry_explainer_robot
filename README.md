## robot_car_meacnum

# ros2
MAKEFLAGS="-j1" colcon build --symlink-install --allow-overriding mecanum_drive_controller --packages-select 
MAKEFLAGS="-j2" colcon build --symlink-install --packages-select mecanum_wheel_chassis_hw
sudo apt update && sudo apt install -y \
    ninja-build \
    build-essential \
    gcc \
    g++ \
    cmake \
    ccache


colcon build --symlink-install --parallel-workers $(nproc) \
             --cmake-args -GNinja -DCMAKE_BUILD_TYPE=Release \
                          -DBUILD_TESTING=OFF \
                          -DCMAKE_CXX_COMPILER_LAUNCHER=ccache

ros2 pkg create --build-type ament_python --license MIT  <package_name> [options]
ros2 pkg create --build-type ament_cmake --license MIT  <package_name> [options]

# gazebo
sudo apt-get install lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-harmonic

colcon build --symlink-install --packages-select name

# 礼貌通知 ROS 节点自杀
ros2 node list 2>/dev/null | xargs -I {} ros2 node kill {} 2>/dev/null

# 硬杀 Gazebo + ROS + Ruby 残留
pkill -9 -f "gz sim"
pkill -9 -f "gazebo"
pkill -9 -f "ruby"

pkill -9 -f "slam_toolbox"
pkill -9 -f "rviz2"

# 确认已清空
ros2 node list 2>/dev/null || echo "All ROS nodes killed"

ros2 topic info /tf -v | grep "Node name"

# 命令行参数：输入话题  输出话题  [-f frame_id]
ros2 run navigation twist_stamper /nav2_controller_raw_cmd /mecanum_drive_controller/reference -f base_link

# 通讯
# ROS_DOMAIN_ID=1 双方id==1
echo "export ROS_DOMAIN_ID=1" >> ~/.bashrc

#   共享文件夹
sudo vim /etc/fstab
.host:/ /mnt/hgfs fuse.vmhgfs-fuse allow_other,uid=1000,gid=1000,umask=022 0 0



deactivate

source ~/mpu6050_env/bin/activate


# ros2_control

| 查看所有已加载控制器              `ros2 control list_controllers`
| 查看硬件接口列表                  `ros2 control list_hardware_interfaces`
| 实时显示控制器状态                `ros2 control list_controllers --watch`
| 启动/停止/重启控制器              `ros2 control switch_controllers --start joint_state_broadcaster --stop forward_position_controller`
| 加载新控制器（不重启节点）         `ros2 control load_controller <name>`
| 卸载控制器                        `ros2 control unload_controller <name>`
| 把控制器参数写到 YAML 并保存       `ros2 control list_controllers -v`
| 可视化控制链                      `ros2 control view_controller_chains`

# 串口调试的工具
cutecom  

# plot tool topic data
/rr_speed_pid/controller_state/dof_states[0]/feedback


# action 
ros2 action send_goal /position_tracking_controller/to_pose \
  robot_msgs/action/ToPose \
  "target_pose:
    header:
      stamp: {sec: 0, nanosec: 0}
      frame_id: odom
    pose:
      position: {x: 1.0, y: 0.5, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"


ros2 launch robot_car_mecanum robot_ros2_control.launch.py  start_position_ctrl:=true


sudo apt install ros-${ROS_DISTRO}-slam-toolbox
sudo apt install ros-${ROS_DISTRO}-turtlebot3*

echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc

# toobox
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py

ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True


# save map
ros2 run nav2_map_server map_saver_cli -f ~/maps/

# topic
//发布话题velocity_controller/commands 类型std_msgs/msg/Float64MultiArray
ros2 topic pub --rate 5 velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [1.0,]}" 

# topic
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"


# vnc

vncserver -kill :3	//删掉vncserver5903端口监听
sudo systemctl status vncserver@:1.service //查看状态

sudo rm /etc/systemd/system/vncserver@:3.service    //删除服务

sudo systemctl daemon-reload    //重新加载服务

vncserver -geometry 1280x720 -depth 16 :1 -localhost no //启动vnc服务器


minicom -D /dev/ttyAMA0 -b 115200

# 启动虚拟环境
source ~/workspace/rosboardev/bin/activate

deactivate rosboardev

# 启动web服务器（必须在对应html的文件夹下面启动）
python3 -m http.server 9999

# 启动桥接
ros2 launch rosbridge_server rosbridge_websocket_launch.xml delay_between_messages 0.0
ros2 run rosbridge_server rosbridge_websocket

# 设置右后轮的比例增益
ros2 param set rr_speed_pid gains.right_rear_wheel_joint.p 0.8

# 设置右后轮的积分增益
ros2 param set rr_speed_pid gains.right_rear_wheel_joint.i 0.2

# 设置右后轮的微分增益
ros2 param set rr_speed_pid gains.right_rear_wheel_joint.d 0.05

# 设置积分限制
ros2 param set rr_speed_pid gains.right_rear_wheel_joint.i_clamp_max 1.0
ros2 param set rr_speed_pid gains.right_rear_wheel_joint.i_clamp_min -1.0