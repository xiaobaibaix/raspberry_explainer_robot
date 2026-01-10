#include "mpu6050_hw/mpu6050_hw.hpp"
#include <cstring>
#include <algorithm>

namespace mpu6050_hw
{

  hardware_interface::CallbackReturn MPU6050HW::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
  {
    if (SensorInterface::on_init(params) != CallbackReturn::SUCCESS)
      return CallbackReturn::ERROR;
    const std::unordered_map<std::string, std::string> &hw_params = 
      params.hardware_info.hardware_parameters;

    auto i2c_file_it = hw_params.find("i2c_file_name");
    if (i2c_file_it == hw_params.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("MPU6050HW"), "错误：URDF 中未配置 i2c_file_name 参数！");
      return CallbackReturn::ERROR;
    }
    i2c_filename_ = i2c_file_it->second;  // 取出键对应的值

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface>
  MPU6050HW::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> si;
    // 用 URDF 里的完整接口名  <-- 关键
    auto add = [&](const std::string &suffix, double *ptr) {
      si.emplace_back("mpu6050", suffix, ptr);
    };

    add("orientation.w", &orientation_[0]);
    add("orientation.x", &orientation_[1]);
    add("orientation.y", &orientation_[2]);
    add("orientation.z", &orientation_[3]);
    add("angular_velocity.x", &angular_velocity_[0]);
    add("angular_velocity.y", &angular_velocity_[1]);
    add("angular_velocity.z", &angular_velocity_[2]);
    add("linear_acceleration.x", &ax_);
    add("linear_acceleration.y", &ay_);
    add("linear_acceleration.z", &az_);
    return si;
  }

  hardware_interface::CallbackReturn MPU6050HW::on_activate(const rclcpp_lifecycle::State &)
  {
    i2c_fd_ = open(i2c_filename_.c_str(), O_RDWR);
    if (i2c_fd_ < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("MPU6050HW"), "Cannot open %s", i2c_filename_.c_str());
      return CallbackReturn::ERROR;
    }
    if (ioctl(i2c_fd_, I2C_SLAVE, MPU6050_ADDR) < 0) {
      RCLCPP_ERROR(rclcpp::get_logger("MPU6050HW"), "Unable to set I2C address");
      return CallbackReturn::ERROR;
    }
    i2c_write(PWR_MGMT_1, 0x00);   // 唤醒
    i2c_write(GYRO_CONFIG,  0x00);  // ±250 °/s
    i2c_write(ACCEL_CONFIG, 0x00);  // ±2 g
    RCLCPP_INFO(rclcpp::get_logger("MPU6050HW"), "MPU6050 activated");
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn MPU6050HW::on_deactivate(const rclcpp_lifecycle::State &)
  {
    if (i2c_fd_ >= 0) ::close(i2c_fd_);
    i2c_fd_ = -1;
    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type MPU6050HW::read(const rclcpp::Time &,const rclcpp::Duration & period)
  {
    uint8_t buf[14];
    if (!i2c_read(ACCEL_XOUT_H, buf, 14))
      return hardware_interface::return_type::OK;

    // 原始数据 16 bit 有符号
    auto convert = [&](int hi, int lo) -> double {
      int16_t raw = (static_cast<int16_t>(buf[hi]) << 8) | buf[lo];
      return static_cast<double>(raw);
    };

    // 量程换算
    ax_ = convert(0, 1) / 16384.0 * 9.80665;  // m/s²
    ay_ = convert(2, 3) / 16384.0 * 9.80665;
    az_ = convert(4, 5) / 16384.0 * 9.80665;
    gx_ = convert(8, 9)  / 131.0 * M_PI / 180.0;  // rad/s
    gy_ = convert(10,11) / 131.0 * M_PI / 180.0;
    gz_ = convert(12,13) / 131.0 * M_PI / 180.0;

    complementary_filter(period.seconds());

    // 填写角速度
    angular_velocity_[0] = gx_;
    angular_velocity_[1] = gy_;
    angular_velocity_[2] = gz_;
    return hardware_interface::return_type::OK;
  }

  bool MPU6050HW::i2c_write(uint8_t reg, uint8_t val)
  {
    uint8_t out[2] = {reg, val};
    return ::write(i2c_fd_, out, 2) == 2;
  }

  bool MPU6050HW::i2c_read(uint8_t reg, uint8_t * buf, size_t len)
  {
    if (::write(i2c_fd_, &reg, 1) != 1) return false;
    return ::read(i2c_fd_, buf, len) == static_cast<ssize_t>(len);
  }

  void MPU6050HW::complementary_filter(double dt)
  {
    // 加速度计算倾角
    double rollAcc  = std::atan2(ay_, az_);
    double pitchAcc = std::atan2(-ax_, std::sqrt(ay_ * ay_ + az_ * az_));

    // 一阶互补滤波
    double alpha = 0.98;
    roll_  = alpha * (roll_  + gx_ * dt) + (1 - alpha) * rollAcc;
    pitch_ = alpha * (pitch_ + gy_ * dt) + (1 - alpha) * pitchAcc;

    //  yaw 积分
    double yaw = gz_ * dt;  // 无磁罗盘，仅相对
    // 欧拉→四元数
    double cy = cos(yaw * 0.5), sy = sin(yaw * 0.5);
    double cr = cos(roll_ * 0.5), sr = sin(roll_ * 0.5);
    double cp = cos(pitch_ * 0.5), sp = sin(pitch_ * 0.5);

    orientation_[0] = cy * cr * cp + sy * sr * sp;  // w
    orientation_[1] = cy * sr * cp - sy * cr * sp;  // x
    orientation_[2] = cy * cr * sp + sy * sr * cp;  // y
    orientation_[3] = sy * cr * cp - cy * sr * sp;  // z
  }

}  // namespace mpu6050_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mpu6050_hw::MPU6050HW, hardware_interface::SensorInterface)