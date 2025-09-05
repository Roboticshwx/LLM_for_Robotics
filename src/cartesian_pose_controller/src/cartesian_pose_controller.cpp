#include <cartesian_pose_controller/cartesian_pose_controller.hpp>
#include <cartesian_pose_controller/default_robot_behavior_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

using namespace std::chrono_literals;

namespace cartesian_pose_controller {

controller_interface::InterfaceConfiguration
CartesianPoseController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_command_interface_names();

  return config;
}

controller_interface::InterfaceConfiguration
CartesianPoseController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();

  return config;
}

controller_interface::return_type CartesianPoseController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {

  // 获取机械臂初始位姿
  if (initialization_flag_) {
    // Get initial orientation and translation
    std::tie(orientation_, position_) =
        franka_cartesian_pose_->getInitialOrientationAndTranslation();
    initialization_flag_ = false;
  // 设置运动参数
  elapsed_time_ = 0.0;  // 重置计数器

  target_displacement_ = 0.05; // 目标位移5cm
  motion_duration_ = 3.0;  // 运动时间设置为3s
  }

  elapsed_time_ = elapsed_time_ + trajectory_period_; // 跟踪轨迹运行时间

  Eigen::Quaterniond new_orientation;
  Eigen::Vector3d new_position;

  new_position = position_;
  new_orientation = orientation_;

  if (elapsed_time_ < motion_duration_) {
    double factor = (1.0 - std::cos(M_PI * elapsed_time_ / motion_duration_)) / 2.0;

    new_position(2) = position_(2) + target_displacement_ * factor; // x轴方向运动
  } else {
    new_position(2) = position_(2) + target_displacement_;
  }

  if (franka_cartesian_pose_->setCommand(new_orientation, new_position)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_FATAL(get_node()->get_logger(),
                  "Set command failed. Did you activate the elbow command interface?");
    return controller_interface::return_type::ERROR;
  }
}

CallbackReturn CartesianPoseController::on_init() {
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated_));

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

  auto future_result = client->async_send_request(request);
  future_result.wait_for(1000ms);

  auto success = future_result.get();
  if (!success) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianPoseController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;
  elapsed_time_ = 0.0;
  franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

}  // namespace cartesian_pose_controller
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cartesian_pose_controller::CartesianPoseController,
                        controller_interface::ControllerInterface)