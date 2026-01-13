#include "position_tracking_controller/position_tracking_controller.hpp"
#include "angles/angles.h"
#include "pluginlib/class_list_macros.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "rclcpp/rclcpp.hpp"

namespace position_tracking_controller
{
    controller_interface::CallbackReturn PositionTrackingController::on_init()
    {
        reference_interfaces_.resize(3);
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration PositionTrackingController::command_interface_configuration() const
    {
        return {
            controller_interface::interface_configuration_type::INDIVIDUAL,
            {"mecanum_x_position_pid/mecanum_drive_controller/linear/x/position",
             "mecanum_y_position_pid/mecanum_drive_controller/linear/y/position",
             "mecanum_z_angle_pid/mecanum_drive_controller/angular/z/position"}};
    }

    controller_interface::InterfaceConfiguration PositionTrackingController::state_interface_configuration() const
    {
        return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::NONE, {}};
    }

    controller_interface::CallbackReturn PositionTrackingController::on_configure(
        const rclcpp_lifecycle::State &previous_state)
    {

        (void)previous_state;

        // 初始目标设为 0
        geometry_msgs::msg::Pose2D p;
        p.x = p.y = p.theta = 0.0;
        current_pose_.writeFromNonRT(p);
        target_pose_.writeFromNonRT(p);

        // Action server
        action_server_ = rclcpp_action::create_server<ControllerReferenceMsg>(
            get_node(), "~/to_pose",
            std::bind(&PositionTrackingController::handle_goal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&PositionTrackingController::handle_cancel, this,
                      std::placeholders::_1),
            std::bind(&PositionTrackingController::handle_accepted, this,
                      std::placeholders::_1));

        // Odom 订阅
        odom_sub_ = get_node()->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", rclcpp::SystemDefaultsQoS(),///mecanum_drive_controller/odometry
            // "mecanum_drive_controller/odometry", rclcpp::SystemDefaultsQoS(),///mecanum_drive_controller/odometry
            std::bind(&PositionTrackingController::odom_callback, this,
                      std::placeholders::_1));

        feedback_x_pub_ = get_node()->create_publisher<ControllerFeedbackMsg>(
            "mecanum_x_position_pid/measured_state", rclcpp::SystemDefaultsQoS());
        feedback_y_pub_ = get_node()->create_publisher<ControllerFeedbackMsg>(
            "mecanum_y_position_pid/measured_state", rclcpp::SystemDefaultsQoS());
        feedback_z_pub_ = get_node()->create_publisher<ControllerFeedbackMsg>(
            "mecanum_z_angle_pid/measured_state", rclcpp::SystemDefaultsQoS());

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PositionTrackingController::on_activate(
        const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        {
            auto it = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [](const auto &ci)
                {
                    return ci.get_name() == "mecanum_x_position_pid/mecanum_drive_controller/linear/x/position";
                });
            if (it == command_interfaces_.end())
            {
                std::cout << "x interface not found!" << std::endl;
                return CallbackReturn::ERROR;
            }
            cmd_pos_.emplace_back(*it);
        }
        {
            auto it = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [](const auto &ci)
                {
                    return ci.get_name() == "mecanum_y_position_pid/mecanum_drive_controller/linear/y/position";
                });
            if (it == command_interfaces_.end())
            {
                std::cout << "y interface not found!" << std::endl;
                return CallbackReturn::ERROR;
            }
            cmd_pos_.emplace_back(*it);
        }
        {
            auto it = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(),
                [](const auto &ci)
                {
                    return ci.get_name() == "mecanum_z_angle_pid/mecanum_drive_controller/angular/z/position";
                });
            if (it == command_interfaces_.end())
            {
                std::cout << "z interface not found!" << std::endl;
                return CallbackReturn::ERROR;
            }
            cmd_pos_.emplace_back(*it);
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PositionTrackingController::on_deactivate(
        const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type PositionTrackingController::update_reference_from_subscribers(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        // 把“目标位姿”写到 reference 接口
        auto tgt = target_pose_.readFromRT();
        bool isok = cmd_pos_[0].get().set_value(tgt->x);
        if (!isok)
        {
            std::cout << "set x failed" << std::endl;
        }
        isok = cmd_pos_[1].get().set_value(tgt->y);
        if (!isok)
        {
            std::cout << "set y failed" << std::endl;
        }
        isok = cmd_pos_[2].get().set_value(tgt->theta);
        if (!isok)
        {
            std::cout << "set theta failed" << std::endl;
        }
        return controller_interface::return_type::OK;
    }

    controller_interface::return_type PositionTrackingController::update_and_write_commands(
        const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        (void)time;
        (void)period;
        // 把“当前位姿”写到 reference 接口，供下游 PID 读取做差
        auto cur = target_pose_.readFromRT();
        bool isok = cmd_pos_[0].get().set_value(cur->x); // 注意：这里不会冲突，下游 PID 读的是另外 3 个接口
        if (!isok)
        {
            std::cout << "set x failed" << std::endl;
        }
        isok = cmd_pos_[1].get().set_value(cur->y);
        if (!isok)
        {
            std::cout << "set y failed" << std::endl;
        }
        isok = cmd_pos_[2].get().set_value(cur->theta);
        if (!isok)
        {
            std::cout << "set theta failed" << std::endl;
        }
        return controller_interface::return_type::OK;
    }

    std::vector<hardware_interface::CommandInterface>
    PositionTrackingController::on_export_reference_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> cmd_ifs;
        cmd_ifs.emplace_back(
            get_node()->get_name(), "linear/x/position", &tgt_x_);
        cmd_ifs.emplace_back(
            get_node()->get_name(), "linear/y/position", &tgt_x_);
        cmd_ifs.emplace_back(
            get_node()->get_name(), "angular/z/position", &tgt_x_);
        return cmd_ifs;
    }

    // ---------------- 以下常规实现 ----------------
    void PositionTrackingController::odom_callback(
        const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::Pose2D p;
        p.x = msg->pose.pose.position.x;
        p.y = msg->pose.pose.position.y;
        p.theta = tf2::getYaw(msg->pose.pose.orientation);
        // 发布反馈
        ControllerFeedbackMsg fb_msg;
        fb_msg.dof_names.resize(1);
        fb_msg.values.resize(1);
        fb_msg.values_dot.resize(1);

        fb_msg.set__values_dot({0.0}); // 必须初始化，否则发布会报错

        fb_msg.dof_names[0] = "mecanum_drive_controller/linear/x";
        fb_msg.values[0] = p.x;
        feedback_x_pub_->publish(fb_msg);
        fb_msg.dof_names[0] = "mecanum_drive_controller/linear/y";
        fb_msg.values[0] = p.y;
        feedback_y_pub_->publish(fb_msg);
        fb_msg.dof_names[0] = "mecanum_drive_controller/angular/z";
        fb_msg.values[0] = p.theta;
        feedback_z_pub_->publish(fb_msg);
        // 写实时缓存
        current_pose_.writeFromNonRT(p);
    }

    rclcpp_action::GoalResponse
    PositionTrackingController::handle_goal(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const ControllerReferenceMsg::Goal> goal)
    {
        // 直接覆盖目标
        auto goal_ptr = goal.get();
        geometry_msgs::msg::Pose2D p;
        p.x = goal_ptr->target_pose.pose.position.x;
        p.y = goal_ptr->target_pose.pose.position.y;
        p.theta = tf2::getYaw(goal_ptr->target_pose.pose.orientation);
        target_pose_.writeFromNonRT(p);
        std::cout<<"New goal received: "
                 << "x=" << p.x << ", y=" << p.y << ", theta=" << p.theta << std::endl;

        
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse
    PositionTrackingController::handle_cancel(
        const std::shared_ptr<GoalHandle>)
    {
        std::cout << "Goal cancel request received." << std::endl;
        auto current_pose = current_pose_.readFromRT();

        target_pose_.writeFromNonRT(*current_pose); // 取消时把目标设为当前位置
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void PositionTrackingController::handle_accepted(
        const std::shared_ptr<GoalHandle> goal_handle)
    {

        std::thread{[goal_handle, this]()
                    { execute_goal(goal_handle); }}
            .detach();
    }

    void PositionTrackingController::execute_goal(
        std::shared_ptr<GoalHandle> goal_handle)
    {
        // 1. 读目标
        auto goal = goal_handle->get_goal();

        // 2. 周期跑控制/跟踪
        auto result = std::make_shared<ControllerReferenceMsg::Result>();
        rclcpp::Rate loop_rate(50);
        std::cout << "Starting to track to target pose..." << std::endl;
        while (rclcpp::ok() && !goal_handle->is_canceling())
        {
            //  做控制，生成 feedback
            auto feedback = std::make_shared<ControllerReferenceMsg::Feedback>();
            feedback->current_pose.header.stamp = get_node()->now();
            feedback->current_pose.header.frame_id = "odom";

            auto cur = current_pose_.readFromRT();
            feedback->current_pose.pose.position.x = cur->x;
            feedback->current_pose.pose.position.y = cur->y;
            feedback->current_pose.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, cur->theta); // 绕 Z 轴旋转
            // feedback->current_pose.pose.orientation = tf2::toMsg<tf2::Quaternion, geometry_msgs::msg::Quaternion>(q);
            tf2::convert(q, feedback->current_pose.pose.orientation);

            double dx = goal->target_pose.pose.position.x - cur->x;
            double dy = goal->target_pose.pose.position.y - cur->y;
            double dist_remain = std::hypot(dx, dy);
            feedback->distance_remaining = static_cast<float>(dist_remain);

            goal_handle->publish_feedback(feedback);
            loop_rate.sleep();

            if (dist_remain < 0.04)
            {
                result->success = true;
                break;
            }
        }
        std::cout << "Finished tracking to target pose." << std::endl;
        // 3. 结束
        if (goal_handle->is_canceling())
        {
            result.get()->success = false;
            goal_handle->canceled(result);
        }
        else
        {
            goal_handle->succeed(result);
        }
    }

} // namespace position_tracking_controller

PLUGINLIB_EXPORT_CLASS(
    position_tracking_controller::PositionTrackingController,
    controller_interface::ChainableControllerInterface)
