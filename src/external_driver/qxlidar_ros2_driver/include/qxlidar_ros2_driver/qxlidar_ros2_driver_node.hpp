#ifndef SCAN_NODE_HPP_
#define SCAN_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "lidar.h"          // 裸机 Dev 类
#include <thread>
#include <atomic>

class ScanNode : public rclcpp::Node
{
public:
  explicit ScanNode(const rclcpp::NodeOptions & options);
  ~ScanNode()override;
private:
  void pollLoop();

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  std::thread worker_;
  std::atomic<bool> running_{true};

  // 参数
  std::string port_;
  int         baud_;
  std::string frame_id_;
  bool        inverted_;
  double      range_min_;
  double      range_max_;
  int         scan_size_;
  double      angle_min_, angle_max_, angle_increment_;

  // 裸机设备对象
  Dev device_;
};

#endif  // SCAN_NODE_HPP_