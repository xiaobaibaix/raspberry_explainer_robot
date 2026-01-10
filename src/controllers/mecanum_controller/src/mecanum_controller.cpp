// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0 
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Bence Magyar, Enrique Fernández, Manuel Meraz
 */

#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include "mecanum_controller/mecanum_controller.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

namespace
{
constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
constexpr auto DEFAULT_ODOMETRY_TOPIC = "~/odom";
constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
}  // namespace

namespace mecanum_controller
{
using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using lifecycle_msgs::msg::State;

MecanumController::MecanumController() : controller_interface::ControllerInterface() {}

const char * MecanumController::feedback_type() const
{
  return params_.position_feedback ? HW_IF_POSITION : HW_IF_VELOCITY;
}

controller_interface::CallbackReturn MecanumController::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration MecanumController::command_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : params_.wheel_names)
  {
    conf_names.push_back(joint_name + "/" + HW_IF_VELOCITY);
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration MecanumController::state_interface_configuration() const
{
  std::vector<std::string> conf_names;
  for (const auto & joint_name : params_.wheel_names)
  {
    conf_names.push_back(joint_name + "/" + feedback_type());
  }
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::return_type MecanumController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  auto logger = get_node()->get_logger();
  if (get_lifecycle_state().id() == State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  // FIX: use new get() interface
  std::shared_ptr<TwistStamped> last_command_msg;
  received_velocity_msg_ptr_.get(
    [&](const auto & ptr) { last_command_msg = ptr; });

  if (last_command_msg == nullptr)
  {
    RCLCPP_WARN(logger, "Velocity message received was a nullptr.");
    return controller_interface::return_type::ERROR;
  }

  const auto age_of_last_command = time - last_command_msg->header.stamp;
  if (age_of_last_command > cmd_vel_timeout_)
  {
    last_command_msg->twist.linear.x  = 0.0;
    last_command_msg->twist.linear.y  = 0.0;
    last_command_msg->twist.angular.z = 0.0;
  }

  TwistStamped command = *last_command_msg;
  double & linear_x_command = command.twist.linear.x;
  double & linear_y_command = command.twist.linear.y;
  double & angular_command  = command.twist.angular.z;

  previous_update_timestamp_ = time;

  const double wheel_separation_x = params_.wheel_separation_x;
  const double wheel_separation_y = params_.wheel_separation_y;
  const double wheel_radius       = params_.wheel_radius;

  if (params_.open_loop)
  {
    odometry_.updateOpenLoop(linear_x_command, linear_y_command, angular_command, time);
  }
  else
  {
    std::vector<double> wheel_feedback_means;
    for (size_t index = 0; index < params_.wheel_names.size(); ++index)
    {
      // FIX: use get_optional()
      const auto feedback_opt = registered_wheel_handles_[index].feedback.get().get_optional();
      if (!feedback_opt)
      {
        RCLCPP_ERROR(logger, "Wheel %s feedback is invalid", feedback_type());
        return controller_interface::return_type::ERROR;
      }
      const double wheel_feedback = *feedback_opt;

      if (std::isnan(wheel_feedback))
      {
        RCLCPP_ERROR(
          logger, "The wheel %s is invalid for index [%zu]", feedback_type(), index);
        return controller_interface::return_type::ERROR;
      }
      wheel_feedback_means.push_back(wheel_feedback);
    }

    if (params_.position_feedback)
    {
      odometry_.update(
        wheel_feedback_means[0], wheel_feedback_means[1],
        wheel_feedback_means[2], wheel_feedback_means[3], time);
    }
    else
    {
      const double dt = period.seconds();
      odometry_.updateFromVelocity(
        wheel_feedback_means[0] * wheel_radius * dt,
        wheel_feedback_means[1] * wheel_radius * dt,
        wheel_feedback_means[2] * wheel_radius * dt,
        wheel_feedback_means[3] * wheel_radius * dt, time);
    }
  }

  tf2::Quaternion orientation;
  orientation.setRPY(0.0, 0.0, odometry_.getHeading());

  bool should_publish = false;
  try
  {
    if (previous_publish_timestamp_ + publish_period_ < time)
    {
      previous_publish_timestamp_ += publish_period_;
      should_publish = true;
    }
  }
  catch (const std::runtime_error &)
  {
    previous_publish_timestamp_ = time;
    should_publish = true;
  }

  if (should_publish)
  {
    if (realtime_odometry_publisher_->trylock())
    {
      auto & odometry_message = realtime_odometry_publisher_->msg_;
      odometry_message.header.stamp            = time;
      odometry_message.pose.pose.position.x    = odometry_.getX();
      odometry_message.pose.pose.position.y    = odometry_.getY();
      odometry_message.pose.pose.orientation.x = orientation.x();
      odometry_message.pose.pose.orientation.y = orientation.y();
      odometry_message.pose.pose.orientation.z = orientation.z();
      odometry_message.pose.pose.orientation.w = orientation.w();
      odometry_message.twist.twist.linear.x    = odometry_.getLinearX();
      odometry_message.twist.twist.linear.y    = odometry_.getLinearY();
      odometry_message.twist.twist.angular.z   = odometry_.getAngular();
      realtime_odometry_publisher_->unlockAndPublish();
    }

    if (params_.enable_odom_tf && realtime_odometry_transform_publisher_->trylock())
    {
      auto & transform = realtime_odometry_transform_publisher_->msg_.transforms.front();
      transform.header.stamp            = time;
      transform.transform.translation.x = odometry_.getX();
      transform.transform.translation.y = odometry_.getY();
      transform.transform.rotation.x    = orientation.x();
      transform.transform.rotation.y    = orientation.y();
      transform.transform.rotation.z    = orientation.z();
      transform.transform.rotation.w    = orientation.w();
      realtime_odometry_transform_publisher_->unlockAndPublish();
    }
  }

  auto & last_command           = previous_commands_.back().twist;
  auto & second_to_last_command = previous_commands_.front().twist;
  limiter_linear_x_->limit(
    linear_x_command, last_command.linear.x,
    second_to_last_command.linear.x, period.seconds());

  limiter_linear_y_->limit(
    linear_y_command, last_command.linear.y,
    second_to_last_command.linear.y, period.seconds());

  limiter_angular_->limit(
    angular_command, last_command.angular.z,
    second_to_last_command.angular.z, period.seconds());

  previous_commands_.pop();
  previous_commands_.emplace(command);

  if (publish_limited_velocity_ && realtime_limited_velocity_publisher_->trylock())
  {
    auto & limited_velocity_command = realtime_limited_velocity_publisher_->msg_;
    limited_velocity_command.header.stamp = time;
    limited_velocity_command.twist        = command.twist;
    realtime_limited_velocity_publisher_->unlockAndPublish();
  }

  const double velocity_front_left =
    (linear_x_command - linear_y_command - (wheel_separation_x + wheel_separation_y) * angular_command) / wheel_radius;
  const double velocity_front_right =
    (linear_x_command + linear_y_command + (wheel_separation_x + wheel_separation_y) * angular_command) / wheel_radius;
  const double velocity_rear_left =
    (linear_x_command + linear_y_command - (wheel_separation_x + wheel_separation_y) * angular_command) / wheel_radius;
  const double velocity_rear_right =
    (linear_x_command - linear_y_command + (wheel_separation_x + wheel_separation_y) * angular_command) / wheel_radius;

  // FIX: nodiscard 检查
  [[maybe_unused]] bool ok1 = registered_wheel_handles_[0].velocity.get().set_value(velocity_front_left);
  [[maybe_unused]] bool ok2 = registered_wheel_handles_[1].velocity.get().set_value(velocity_front_right);
  [[maybe_unused]] bool ok3 = registered_wheel_handles_[2].velocity.get().set_value(velocity_rear_left);
  [[maybe_unused]] bool ok4 = registered_wheel_handles_[3].velocity.get().set_value(velocity_rear_right);

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn MecanumController::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();
  if (param_listener_->is_old(params_))
  {
    params_ = param_listener_->get_params();
    RCLCPP_INFO(logger, "Parameters were updated");
  }

  if (params_.wheel_names.size() != 4)
  {
    RCLCPP_ERROR(logger, "Expected 4 wheels, got %zu", params_.wheel_names.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  const double wheel_separation_x = params_.wheel_separation_x / 2.0;
  const double wheel_separation_y = params_.wheel_separation_y / 2.0;
  const double wheel_radius       = params_.wheel_radius;

  odometry_.setWheelParams(wheel_separation_x, wheel_separation_y, wheel_radius);
  odometry_.setVelocityRollingWindowSize(static_cast<size_t>(params_.velocity_rolling_window_size));

  cmd_vel_timeout_            = std::chrono::milliseconds(static_cast<int>(params_.cmd_vel_timeout * 1000.0));
  publish_limited_velocity_   = params_.publish_limited_velocity;
  use_stamped_vel_            = params_.use_stamped_vel;

  limiter_linear_x_ = std::make_unique<SpeedLimiter>(
    params_.linear.x.min_velocity, params_.linear.x.max_velocity,
    params_.linear.x.max_acceleration_reverse, params_.linear.x.max_acceleration,
    params_.linear.x.max_deceleration, params_.linear.x.max_deceleration_reverse,
    params_.linear.x.min_jerk, params_.linear.x.max_jerk);

  limiter_linear_y_ = std::make_unique<SpeedLimiter>(
    params_.linear.y.min_velocity, params_.linear.y.max_velocity,
    params_.linear.y.max_acceleration_reverse, params_.linear.y.max_acceleration,
    params_.linear.y.max_deceleration, params_.linear.y.max_deceleration_reverse,
    params_.linear.y.min_jerk, params_.linear.y.max_jerk);

  limiter_angular_ = std::make_unique<SpeedLimiter>(
    params_.angular.z.min_velocity, params_.angular.z.max_velocity,
    params_.angular.z.max_acceleration_reverse, params_.angular.z.max_acceleration,
    params_.angular.z.max_deceleration, params_.angular.z.max_deceleration_reverse,
    params_.angular.z.min_jerk, params_.angular.z.max_jerk);

  if (!reset()) return controller_interface::CallbackReturn::ERROR;

  if (publish_limited_velocity_)
  {
    limited_velocity_publisher_ = get_node()->create_publisher<TwistStamped>(
      DEFAULT_COMMAND_OUT_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_limited_velocity_publisher_ =
      std::make_shared<realtime_tools::RealtimePublisher<TwistStamped>>(limited_velocity_publisher_);
  }

  const TwistStamped empty_twist;
  // FIX: new set() interface
  received_velocity_msg_ptr_.set(
    [&](auto & ptr) { ptr = std::make_shared<TwistStamped>(empty_twist); });

  previous_commands_.emplace(empty_twist);
  previous_commands_.emplace(empty_twist);

  if (use_stamped_vel_)
  {
    velocity_command_subscriber_ = get_node()->create_subscription<TwistStamped>(
      DEFAULT_COMMAND_TOPIC, rclcpp::SystemDefaultsQoS(),
      [this](std::shared_ptr<TwistStamped> msg)
      {
        if (!subscriber_is_active_)
        {
          RCLCPP_WARN(get_node()->get_logger(), "Subscriber inactive, drop command");
          return;
        }
        if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0)
        {
          RCLCPP_WARN_ONCE(
            get_node()->get_logger(),
            "Zero timestamp detected, setting to now (only once)");
          msg->header.stamp = get_node()->get_clock()->now();
        }
        // FIX: new set() interface
        received_velocity_msg_ptr_.set(
          [&](auto & ptr) { ptr = std::move(msg); });

        std::shared_ptr<TwistStamped> cmd;
        received_velocity_msg_ptr_.get([&](const auto & ptr) { cmd = ptr; });

        RCLCPP_WARN_ONCE(
          get_node()->get_logger(),
          "have:(%f %f %f)",
          cmd->twist.linear.x,
          cmd->twist.linear.y,
          cmd->twist.angular.z);
      });
  }
  else
  {
    velocity_command_unstamped_subscriber_ =
      get_node()->create_subscription<Twist>(
        DEFAULT_COMMAND_UNSTAMPED_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](std::shared_ptr<Twist> msg)
        {
          if (!subscriber_is_active_){
            RCLCPP_WARN(get_node()->get_logger(), "Subscriber inactive, drop command");
            return;
          }
          std::shared_ptr<TwistStamped> twist_stamped=std::make_shared<TwistStamped>();

          twist_stamped->twist       = *msg;
          twist_stamped->header.stamp = get_node()->get_clock()->now();

          received_velocity_msg_ptr_.set(
            [&](auto & ptr) { ptr = std::move(twist_stamped); });

          std::shared_ptr<TwistStamped> cmd;
          received_velocity_msg_ptr_.get([&](const auto & ptr) { cmd = ptr; });

          RCLCPP_WARN_ONCE(
            get_node()->get_logger(),
            "not have:(%f %f %f)",
            cmd->twist.linear.x,
            cmd->twist.linear.y,
            cmd->twist.angular.z);

        });          
  }

  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
    DEFAULT_ODOMETRY_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(odometry_publisher_);

  // tf 前缀处理略...
  std::string tf_prefix = "";
  if (params_.tf_frame_prefix_enable)
  {
    tf_prefix = params_.tf_frame_prefix.empty()
                  ? std::string(get_node()->get_namespace())
                  : params_.tf_frame_prefix;
    if (tf_prefix.back() != '/') tf_prefix += "/";
    if (tf_prefix.front() == '/') tf_prefix.erase(0, 1);
  }

  const auto odom_frame_id = tf_prefix + params_.odom_frame_id;
  const auto base_frame_id = tf_prefix + params_.base_frame_id;

  auto & odometry_message = realtime_odometry_publisher_->msg_;
  odometry_message.header.frame_id = odom_frame_id;
  odometry_message.child_frame_id  = base_frame_id;

  publish_rate_ = params_.publish_rate;
  publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);

  odometry_message.twist =
    geometry_msgs::msg::TwistWithCovariance(rosidl_runtime_cpp::MessageInitialization::ALL);
  constexpr size_t N = 6;
  for (size_t i = 0; i < N; ++i)
  {
    const size_t diag = N * i + i;
    odometry_message.pose.covariance[diag]  = params_.pose_covariance_diagonal[i];
    odometry_message.twist.covariance[diag] = params_.twist_covariance_diagonal[i];
  }

  odometry_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
    DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());
  realtime_odometry_transform_publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
      odometry_transform_publisher_);
  auto & tf_msg = realtime_odometry_transform_publisher_->msg_;
  tf_msg.transforms.resize(1);
  auto & odom_tf = realtime_odometry_transform_publisher_->msg_.transforms.front();
  odom_tf.header.frame_id = odom_frame_id;
  odom_tf.child_frame_id  = base_frame_id;

  previous_update_timestamp_ = get_node()->get_clock()->now();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumController::on_activate(
  const rclcpp_lifecycle::State &)
{
  if (configure_wheels(params_.wheel_names, registered_wheel_handles_) ==
      controller_interface::CallbackReturn::ERROR)
    return controller_interface::CallbackReturn::ERROR;

  if (registered_wheel_handles_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Wheel handles empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  is_halted            = false;
  subscriber_is_active_ = true;
  RCLCPP_DEBUG(get_node()->get_logger(), "Activated.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  subscriber_is_active_ = false;
  if (!is_halted) { halt(); is_halted = true; }
  registered_wheel_handles_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn MecanumController::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  return reset() ? controller_interface::CallbackReturn::SUCCESS
                 : controller_interface::CallbackReturn::ERROR;
}

controller_interface::CallbackReturn MecanumController::on_error(
  const rclcpp_lifecycle::State &)
{
  return reset() ? controller_interface::CallbackReturn::SUCCESS
                 : controller_interface::CallbackReturn::ERROR;
}

bool MecanumController::reset()
{
  odometry_.resetOdometry();
  std::queue<TwistStamped> empty;
  std::swap(previous_commands_, empty);
  registered_wheel_handles_.clear();
  subscriber_is_active_ = false;
  velocity_command_subscriber_.reset();
  velocity_command_unstamped_subscriber_.reset();
  // FIX: new set() interface
  received_velocity_msg_ptr_.set([&](auto & ptr) { ptr = nullptr; });
  is_halted = false;
  return true;
}

void MecanumController::halt()
{
  for (auto & wh : registered_wheel_handles_)
    // FIX: nodiscard 检查
    [[maybe_unused]] bool ok = wh.velocity.get().set_value(0.0);
}

controller_interface::CallbackReturn MecanumController::configure_wheels(
  const std::vector<std::string> & wheel_names,
  std::vector<WheelHandle> & registered_handles)
{
  auto logger = get_node()->get_logger();
  if (wheel_names.empty())
  {
    RCLCPP_ERROR(logger, "No wheel names provided");
    return controller_interface::CallbackReturn::ERROR;
  }

  registered_handles.reserve(wheel_names.size());
  for (const auto & name : wheel_names)
  {
    const auto iface_name = feedback_type();
    const auto st_it = std::find_if(
      state_interfaces_.cbegin(), state_interfaces_.cend(),
      [&name, &iface_name](const auto & i)
      {
        return i.get_prefix_name() == name && i.get_interface_name() == iface_name;
      });
    if (st_it == state_interfaces_.cend())
    {
      RCLCPP_ERROR(logger, "Missing state interface for %s", name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    const auto cmd_it = std::find_if(
      command_interfaces_.begin(), command_interfaces_.end(),
      [&name](const auto & i)
      {
        return i.get_prefix_name() == name && i.get_interface_name() == HW_IF_VELOCITY;
      });
    if (cmd_it == command_interfaces_.end())
    {
      RCLCPP_ERROR(logger, "Missing command interface for %s", name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }

    registered_handles.emplace_back(WheelHandle{std::ref(*st_it), std::ref(*cmd_it)});
  }
  return controller_interface::CallbackReturn::SUCCESS;
}
}  // namespace mecanum_controller

#include "class_loader/register_macro.hpp"
CLASS_LOADER_REGISTER_CLASS(
  mecanum_controller::MecanumController, controller_interface::ControllerInterface)