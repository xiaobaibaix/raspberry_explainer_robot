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
#include "mecanum_motor_driver.hpp"

namespace mecanum_wheel_chassis_hw
{
    class MecanumWheelChassisHW : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(MecanumWheelChassisHW)
        MecanumWheelChassisHW() = default;
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

        struct MotorPosition
        {
            int id{};
            int encode{};
        };

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

        std::unique_ptr<MecanumMotorDriver> motor_driver_;

        
    };
}
