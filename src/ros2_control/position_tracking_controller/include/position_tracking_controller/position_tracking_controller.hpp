#ifndef POSITION_TRACKING_CONTROLLER__POSITION_TRACKING_CONTROLLER_HPP_
#define POSITION_TRACKING_CONTROLLER__POSITION_TRACKING_CONTROLLER_HPP_

#include <vector>
#include <string>

#include "controller_interface/chainable_controller_interface.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"

#include "robot_msgs/action/to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include <realtime_tools/realtime_buffer.hpp>   // 用 hpp 后缀
#include "control_msgs/msg/multi_dof_command.hpp"

namespace position_tracking_controller
{

    class PositionTrackingController : public controller_interface::ChainableControllerInterface
    {
    public:
        PositionTrackingController() = default;
        ~PositionTrackingController() override = default;

        controller_interface::CallbackReturn on_init() override;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::return_type update_reference_from_subscribers(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        controller_interface::return_type update_and_write_commands(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

    protected:
        std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
    private://control_msgs/msg/MultiDOFCommand

        using ControllerReferenceMsg = robot_msgs::action::ToPose;
        using ControllerFeedbackMsg = control_msgs::msg::MultiDOFCommand;
        using GoalHandle = rclcpp_action::ServerGoalHandle<ControllerReferenceMsg>;

        void execute_goal(std::shared_ptr<GoalHandle> goal_handle);
    private:

        // Action：仅用来接收“新目标位姿”
        rclcpp_action::Server<ControllerReferenceMsg>::SharedPtr action_server_;

        // 反馈发布
        rclcpp::Publisher<ControllerFeedbackMsg>::SharedPtr feedback_x_pub_;
        rclcpp::Publisher<ControllerFeedbackMsg>::SharedPtr feedback_y_pub_;
        rclcpp::Publisher<ControllerFeedbackMsg>::SharedPtr feedback_z_pub_;

        // Odom 订阅
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        // 实时缓存
        realtime_tools::RealtimeBuffer<geometry_msgs::msg::Pose2D> current_pose_;
        realtime_tools::RealtimeBuffer<geometry_msgs::msg::Pose2D> target_pose_;

        std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> cmd_pos_;
        // 原始指针，用于绑定到 reference 接口 目标值
        double tgt_x_ = 0.0;
        double tgt_y_ = 0.0;
        double tgt_theta_ = 0.0;
        
        // 原始指针，用于绑定到 status 接口 反馈值
        double curr_x_ = 0.0;
        double curr_y_ = 0.0;
        double curr_theta_ = 0.0;

        // Action 回调
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const ControllerReferenceMsg::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandle> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    };

} // namespace position_tracking_controller

#endif // POSITION_TRACKING_CONTROLLER__POSITION_TRACKING_CONTROLLER_HPP_