#include "mecanum_wheel_chassis_hw2/mecanum_wheel_chassis_hw2.hpp"
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>

namespace mecanum_wheel_chassis_hw2
{
    hardware_interface::CallbackReturn MecanumWheelChassisHW2::on_init(
        const hardware_interface::HardwareComponentInterfaceParams &params)
    {
        if (hardware_interface::SystemInterface::on_init(params) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // 初始化日志记录器
        if (info_.hardware_parameters.empty())
        {
            RCLCPP_WARN(rclcpp::get_logger("mecanum_wheel_chassis"), "没有硬件参数，使用默认值");
        }

        // 设置串口参数
        serial_port_ = "/dev/ttyUSB0";
        if (info_.hardware_parameters.find("serial_port") != info_.hardware_parameters.end())
        {
            serial_port_ = info_.hardware_parameters.at("serial_port");
        }

        baud_rate_ = 115200;
        if (info_.hardware_parameters.find("baud_rate") != info_.hardware_parameters.end())
        {
            try
            {
                baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("mecanum_wheel_chassis"), "baud_rate 参数转换失败: %s", e.what());
            }
        }

        // 编码器参数
        encoder_ppr_ = 4096.0;
        if (info_.hardware_parameters.find("encoder_ppr") != info_.hardware_parameters.end())
        {
            try
            {
                encoder_ppr_ = std::stod(info_.hardware_parameters.at("encoder_ppr"));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("mecanum_wheel_chassis"), "encoder_ppr 参数转换失败: %s", e.what());
            }
        }

        // 机械参数
        wheel_radius_ = 0.05;
        if (info_.hardware_parameters.find("wheel_radius") != info_.hardware_parameters.end())
        {
            try
            {
                wheel_radius_ = std::stod(info_.hardware_parameters.at("wheel_radius"));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("mecanum_wheel_chassis"), "wheel_radius 参数转换失败: %s", e.what());
            }
        }

        gear_ratio_ = 1.0;
        if (info_.hardware_parameters.find("gear_ratio") != info_.hardware_parameters.end())
        {
            try
            {
                gear_ratio_ = std::stod(info_.hardware_parameters.at("gear_ratio"));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("mecanum_wheel_chassis"), "gear_ratio 参数转换失败: %s", e.what());
            }
        }

        // 滤波参数
        filter_pos_alpha_ = 0.3;
        if (info_.hardware_parameters.find("filter_position_alpha") != info_.hardware_parameters.end())
        {
            try
            {
                filter_pos_alpha_ = std::stod(info_.hardware_parameters.at("filter_position_alpha"));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("mecanum_wheel_chassis"), "filter.position_alpha 参数转换失败: %s", e.what());
            }
        }

        filter_vel_alpha_ = 0.2;
        if (info_.hardware_parameters.find("filter_velocity_alpha") != info_.hardware_parameters.end())
        {
            try
            {
                filter_vel_alpha_ = std::stod(info_.hardware_parameters.at("filter_velocity_alpha"));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("mecanum_wheel_chassis"), "filter.velocity_alpha 参数转换失败: %s", e.what());
            }
        }

        filter_enable_ = true;
        if (info_.hardware_parameters.find("filter_enable") != info_.hardware_parameters.end())
        {
            try
            {
                filter_enable_ = std::stoi(info_.hardware_parameters.at("filter_enable"));
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(rclcpp::get_logger("mecanum_wheel_chassis"), "filter.enable 参数转换失败: %s", e.what());
            }
        }

        // 初始化数组
        const size_t n = info_.joints.size();
        hw_positions_.assign(n, 0.0);
        hw_velocities_.assign(n, 0.0);
        hw_commands_.assign(n, 0.0);
        command_interface_types_.resize(n);

        // 滤波相关数组
        filtered_positions_.assign(n, 0.0);
        filtered_velocities_.assign(n, 0.0);
        last_encoder_counts_.assign(n, 0);

        frist_readencode = true;
        for (size_t i = 0; i < n; ++i)
        {
            if (!info_.joints[i].command_interfaces.empty())
            {
                command_interface_types_[i] = info_.joints[i].command_interfaces[0].name;
            }
            else
            {
                command_interface_types_[i] = "velocity";
            }
        }
        node_=node_ = std::make_shared<rclcpp::Node>("mecanum_wheel_chassis_hw_node");
        if (!node_) {
            RCLCPP_ERROR(rclcpp::get_logger("mecanum_wheel_chassis"), "无法获取节点");
            return hardware_interface::CallbackReturn::ERROR;
        }
        wheel_speed_pub_=node_->create_publisher<robot_msgs::msg::MotorsState>(
            "chassis_node/set_motor_speed", rclcpp::QoS(10));
        wheel_encoders_client_ = node_->create_client<robot_msgs::srv::GetPWMServoState>(
            "chassis_node/get_motor_encodes");


        RCLCPP_INFO(rclcpp::get_logger("mecanum_wheel_chassis"), 
            "Mecanum轮式底盘硬件接口初始化成功");
        RCLCPP_INFO(rclcpp::get_logger("mecanum_wheel_chassis"), 
            "编码器PPR: %.0f, 轮半径: %.3f, 减速比: %.2f",
            encoder_ppr_, wheel_radius_, gear_ratio_);
        RCLCPP_INFO(rclcpp::get_logger("mecanum_wheel_chassis"), 
            "滤波参数: 位置α=%.2f, 速度α=%.2f, 启用: %s",
            filter_pos_alpha_, filter_vel_alpha_, filter_enable_ ? "是" : "否");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> MecanumWheelChassisHW2::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
            state_interfaces.emplace_back(
                hardware_interface::StateInterface(
                    info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> MecanumWheelChassisHW2::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); ++i)
        {
            const auto &type = command_interface_types_[i];
            command_interfaces.emplace_back(
                hardware_interface::CommandInterface(
                    info_.joints[i].name, type, &hw_commands_[i]));
        }
        return command_interfaces;
    }

    hardware_interface::CallbackReturn MecanumWheelChassisHW2::on_activate(
        const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;

        // 重置状态
        for (size_t i = 0; i < hw_positions_.size(); ++i)
        {
            filtered_positions_[i] = 0.0;
            filtered_velocities_[i] = 0.0;
        }

        frist_readencode = true;

        //等待服务可用
        while (!wheel_encoders_client_->wait_for_service(std::chrono::seconds(2)))
        {
            RCLCPP_ERROR(rclcpp::get_logger("mecanum_wheel_chassis"), "无法连接到编码器服务");
        }

        RCLCPP_INFO(rclcpp::get_logger("mecanum_wheel_chassis"), "硬件接口激活");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MecanumWheelChassisHW2::on_deactivate(
        const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;

        // 停止所有电机
        wheel_speed_pub_->publish(robot_msgs::msg::MotorsState(
            robot_msgs::msg::MotorsState().set__data({
                robot_msgs::msg::MotorState().set__id(1).set__rps(0),
                robot_msgs::msg::MotorState().set__id(2).set__rps(0),
                robot_msgs::msg::MotorState().set__id(3).set__rps(0),
                robot_msgs::msg::MotorState().set__id(4).set__rps(0),
            })
        ));

        frist_readencode = false;
        RCLCPP_INFO(rclcpp::get_logger("mecanum_wheel_chassis"), "硬件接口停用，电机已停止");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type MecanumWheelChassisHW2::read(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        double dt = period.seconds();
        std::array<int32_t, 4> encoders;

        // 通过服务获取编码器数据
        auto request = std::make_shared<robot_msgs::srv::GetPWMServoState::Request>();
        request.get()->set__cmd({
            robot_msgs::msg::GetPWMServoCmd().set__id(1),
            robot_msgs::msg::GetPWMServoCmd().set__id(2),
            robot_msgs::msg::GetPWMServoCmd().set__id(3),
            robot_msgs::msg::GetPWMServoCmd().set__id(4),
        });
        auto result_future = wheel_encoders_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, result_future, std::chrono::milliseconds(5)) == rclcpp::FutureReturnCode::SUCCESS)
        {
            auto resp = result_future.get();

            for(int i=0; i < resp.get()->state.size(); ++i)
            {
                auto pos = resp.get()->state[i].position;
                encoders[i] = pos.at(0);
            }
        }

        // // 计算编码器一圈对应的角度
        // static const double encoder_resolution = 2.0 * M_PI / (encoder_ppr_ * gear_ratio_);
        // static int32_t last_delta_diff[4] = {0, 0, 0, 0};

        // for (size_t i = 0; i < hw_positions_.size(); ++i)
        // {

        //     int32_t encoder_diff;
        //     // if (i == 1 || i == 2)
        //     //     encoders[i] *= -1;

        //     encoder_diff = encoders[i] - last_encoder_counts_[i];
        //     last_encoder_counts_[i] = encoders[i];

        //     // 计算角度变化
        //     double delta_angle = encoder_diff * encoder_resolution;

        //     double raw_position = hw_positions_[i] + delta_angle;

        //     // 位置滤波
        //     filtered_positions_[i] = filter_pos_alpha_ * raw_position +
        //                                 (1.0 - filter_pos_alpha_) * filtered_positions_[i];

        //     // 计算速度
        //     double raw_velocity = delta_angle / dt;

        //     // 速度滤波
        //     filtered_velocities_[i] = filter_vel_alpha_ * raw_velocity +
        //                                 (1.0 - filter_vel_alpha_) * filtered_velocities_[i];

        //     // 更新硬件接口值
        //     hw_positions_[i] = filtered_positions_[i];
        //     hw_velocities_[i] = filtered_velocities_[i];

        //     // 保存当前编码器值用于下一次计算
        //     last_delta_diff[i] = encoder_diff;
        // }

        // RCLCPP_INFO(rclcpp::get_logger("mecanum_wheel_chassis"),
        //             "encode=[%d,%d,%d,%d]",
        //             encoders[0], encoders[1], encoders[2], encoders[3]);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MecanumWheelChassisHW2::write(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;

        std::array<int16_t, 4> pwm_commands = {0};

        for (size_t i = 0; i < hw_commands_.size(); ++i)
        {
            pwm_commands[i] = static_cast<int16_t>(hw_commands_[i]);
            if (i == 1 || i == 2)
                pwm_commands[i] *= -1;
        }
        wheel_speed_pub_->publish(robot_msgs::msg::MotorsState(
            robot_msgs::msg::MotorsState().set__data({
                robot_msgs::msg::MotorState().set__id(1).set__rps(pwm_commands[0]),
                robot_msgs::msg::MotorState().set__id(2).set__rps(pwm_commands[1]),
                robot_msgs::msg::MotorState().set__id(3).set__rps(pwm_commands[2]),
                robot_msgs::msg::MotorState().set__id(4).set__rps(pwm_commands[3]),
            })
        ));
        return hardware_interface::return_type::OK;
    }

} // namespace mecanum_wheel_chassis_hw

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mecanum_wheel_chassis_hw2::MecanumWheelChassisHW2, hardware_interface::SystemInterface)