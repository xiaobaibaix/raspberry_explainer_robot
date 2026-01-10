#ifndef MPU6050_HW__MPU6050_HW_HPP_
#define MPU6050_HW__MPU6050_HW_HPP_

#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/sensor_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cmath>
#include <linux/i2c-dev.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

namespace mpu6050_hw
{

  class MPU6050HW : public hardware_interface::SensorInterface
  {
  public:
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::return_type read(const rclcpp::Time & time,const rclcpp::Duration & period) override;

  private:
    int i2c_fd_ = -1;
    static constexpr uint8_t MPU6050_ADDR = 0x68;
    static constexpr uint8_t PWR_MGMT_1   = 0x6B;
    static constexpr uint8_t GYRO_CONFIG  = 0x1B;
    static constexpr uint8_t ACCEL_CONFIG = 0x1C;
    static constexpr uint8_t ACCEL_XOUT_H = 0x3B;
    // i2c filename
    std::string i2c_filename_;

    // 原始量
    double ax_ = 0, ay_ = 0, az_ = 0;
    double gx_ = 0, gy_ = 0, gz_ = 0;

    // 状态接口用
    std::array<double, 4> orientation_{1, 0, 0, 0};  // wxyz
    std::array<double, 3> angular_velocity_{0, 0, 0};

    // 互补滤波用
    double roll_  = 0;
    double pitch_ = 0;

    bool i2c_write(uint8_t reg, uint8_t val);
    bool i2c_read(uint8_t reg, uint8_t * buf, size_t len);
    void complementary_filter(double dt);
  };

}  // namespace mpu6050_hw

#endif  // MPU6050_HW__MPU6050_HW_HPP_