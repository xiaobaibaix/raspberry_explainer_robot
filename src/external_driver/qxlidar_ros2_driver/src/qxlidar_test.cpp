#include "qxlidar_ros2_driver/qxlidar_ros2_driver_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}