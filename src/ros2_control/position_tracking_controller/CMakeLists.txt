cmake_minimum_required(VERSION 3.8)
project(position_tracking_controller)

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(control_msgs REQUIRED)

add_library(position_tracking_controller SHARED
  src/position_tracking_controller.cpp)

  # 1. 先让 ament 把 include 路径、transitive deps 准备好
ament_target_dependencies(position_tracking_controller
  controller_interface
  pluginlib
  rclcpp
  geometry_msgs
  tf2                    
  tf2_geometry_msgs     
  angles
  nav2_msgs
  action_msgs
  robot_msgs
  control_msgs
)
target_include_directories(position_tracking_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 关键：显式链接 TF2
target_link_libraries(position_tracking_controller
  controller_interface::controller_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  ${geometry_msgs_TARGETS}
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
  angles::angles
  ${nav2_msgs_TARGETS}
  ${action_msgs_TARGETS}
  ${robot_msgs_TARGETS}
  ${control_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(
  controller_interface position_tracking_controller_plugin.xml)

install(TARGETS position_tracking_controller
  EXPORT export_position_tracking_controller
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(position_tracking_controller)
ament_export_dependencies(
  controller_interface
  pluginlib
  rclcpp
  geometry_msgs
  tf2
  tf2_geometry_msgs
  angles
  nav2_msgs
  action_msgs
  robot_msgs
  control_msgs
  )
ament_package()