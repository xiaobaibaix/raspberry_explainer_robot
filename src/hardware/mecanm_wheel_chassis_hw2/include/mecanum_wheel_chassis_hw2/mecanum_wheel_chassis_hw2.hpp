#pragma once
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <algorithm>
#include <chrono>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include <boost/circular_buffer.hpp>

#include "robot_msgs/msg/motors_state.hpp"
#include "robot_msgs/srv/get_pwm_servo_state.hpp"

namespace mecanum_wheel_chassis_hw2
{
    class MecanumWheelChassisHW2 : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(MecanumWheelChassisHW2)
        MecanumWheelChassisHW2() = default;
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams &params) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        std::vector<double> hw_positions_;
        std::vector<double> hw_velocities_;
        std::vector<double> hw_commands_;

        double encoder_ppr_;  // 一圈编码值
        double wheel_radius_; // 轮半径
        double gear_ratio_;   // 齿轮比
        std::string serial_port_;
        double baud_rate_;

        // 滤波参数
        double filter_pos_alpha_;
        double filter_vel_alpha_;
        bool filter_enable_;
        bool frist_readencode;
        bool is_timeout=false;

        // 滤波状态
        std::vector<double> filtered_positions_;
        std::vector<double> filtered_velocities_;

        std::vector<int32_t> last_encoder_counts_;

        std::vector<std::string> command_interface_types_;
        
        rclcpp::Node::SharedPtr node_;

        rclcpp::Publisher<robot_msgs::msg::MotorsState>::SharedPtr wheel_speed_pub_;  // 轮速命令发布者
        rclcpp::Client<robot_msgs::srv::GetPWMServoState>::SharedPtr wheel_encoders_client_; // 编码器读取客户端
    };
}
