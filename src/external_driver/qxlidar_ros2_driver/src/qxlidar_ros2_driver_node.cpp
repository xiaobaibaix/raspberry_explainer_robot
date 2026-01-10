#include "qxlidar_ros2_driver/qxlidar_ros2_driver_node.hpp"
#include "hchead.h"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

ScanNode::ScanNode(const rclcpp::NodeOptions & options)
: Node("scan_node", options), running_(true)  // 初始化running_为true
{
  port_       = declare_parameter<std::string>("port", "/dev/ttyAMA0");
  baud_       = declare_parameter<int>("baud", 115200);
  frame_id_   = declare_parameter<std::string>("frame_id", "laser");
  inverted_   = declare_parameter<bool>("inverted", false);
  range_min_  = declare_parameter<double>("range_min", 0.02);
  range_max_  = declare_parameter<double>("range_max", 12.0);
  scan_size_  = declare_parameter<int>("scan_size", 1440);

  angle_increment_ = 2.0 * M_PI / scan_size_;
  angle_min_       = 0.0;
  angle_max_       = 2.0 * M_PI - angle_increment_;

  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::QoS(10));

  /*  打开串口  */
  char port_path[32];
  snprintf(port_path, sizeof(port_path), "/dev/ttyAMA%d", port_.back() - '0'); // 简易解析数字
  if (device_.Initialize(port_path, baud_, true) != 0) {
    RCLCPP_FATAL(get_logger(), "Open %s failed", port_path);
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(get_logger(), "Lidar ready, publishing /scan");
  worker_ = std::thread(&ScanNode::pollLoop, this);
}

void ScanNode::pollLoop()
{
  std::list<node_info> dataList;
  // 激光雷达常规扫描频率约10Hz，单次循环等待10ms（100Hz）既不丢数据也不占CPU
  const std::chrono::milliseconds loop_delay(10);
  
  while (rclcpp::ok() && running_) {
    dataList.clear();  // 每次循环清空旧数据，避免累积
    device_.GetScanData(dataList, inverted_);   // 与裸机完全一致
    int err = device_.GetLastErrCode();
    
    if (err != 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "GetScanData err=%d", err);
      std::this_thread::sleep_for(loop_delay);  // 出错时也等待
      continue;
    }
    
    if (dataList.size() < 300) {
      // 数据不足时等待，避免空转
      std::this_thread::sleep_for(loop_delay);
      continue;
    }       

    auto scan = sensor_msgs::msg::LaserScan();
    scan.header.stamp    = now();
    scan.header.frame_id = frame_id_;
    scan.angle_min       = angle_min_;
    scan.angle_max       = angle_max_;
    scan.angle_increment = angle_increment_;
    scan.range_min       = range_min_;
    scan.range_max       = range_max_;
    scan.ranges.assign(scan_size_, std::numeric_limits<float>::infinity());
    scan.intensities.assign(scan_size_, 0);

    size_t valid = 0;
    for (const auto & p : dataList) {
      if (!p.isValid) continue;
      double angle_deg = p.angle_q6_checkbit / 64.0f;
      double angle_rad = angle_deg * M_PI / 180.0;
      int    index     = static_cast<int>((angle_rad - angle_min_) / angle_increment_);
      if (index < 0 || index >= scan_size_) continue;
      scan.ranges[index]      = (p.distance_q2 / 4.0f) * 0.001f; // mm→m
      scan.intensities[index] = static_cast<float>(p.syn_quality);
      ++valid;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                         "Published 1 circle: %zu points, valid=%zu", dataList.size(), valid);
    scan_pub_->publish(scan);
    
    // 每次发布数据后也短暂等待，降低循环频率
    std::this_thread::sleep_for(loop_delay);
  }
  device_.Uninit();
}

ScanNode::~ScanNode()
{
  running_ = false;
  if (worker_.joinable()) worker_.join();
}