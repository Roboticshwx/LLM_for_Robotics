#pragma once

#include <Eigen/Dense>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <franka_semantic_components/franka_cartesian_pose_interface.hpp>

#include <functional>
#include <memory>
#include <thread>

#include "communication_interfaces/action/cartesian.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"



using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace cartesian_pose_controller {

class CartesianPoseController : public controller_interface::ControllerInterface {
public:

  using Vector4d = Eigen::Matrix<double, 4, 1>;

  CartesianPoseController() = default;

  using Cartesian = communication_interfaces::action::Cartesian; 
  using GoalHandleCartesian = rclcpp_action::ServerGoalHandle<Cartesian>;

  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                            const rclcpp::Duration& period) override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  // action server回调函数
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const Cartesian::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleCartesian> goal_handle);
  void handle_accepted(
    const std::shared_ptr<GoalHandleCartesian> goal_handle);  // 使用GoalHandleCartesian
  void execute(const std::shared_ptr<GoalHandleCartesian> goal_handle);

private:
  // action server的实例
  rclcpp_action::Server<Cartesian>::SharedPtr action_server_;

  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;

  Eigen::Quaterniond orientation_;
  Eigen::Vector3d position_;

  Eigen::Quaterniond new_orientation;
  Eigen::Vector3d new_position;

  Vector4d cartesian_move_; // 笛卡尔运动参数

  double trajectory_period_{0.001};
  const bool k_elbow_activated_{false};
  bool initialization_flag_{true};

  bool cartesian_flag_{false};
  bool cartesian_finished_{false};

  double elapsed_time_{0.0};
  double motion_duration_;
  double x_displacement_;
  double y_displacement_;
  double z_displacement_;
};
}  // namespace cartesian_pose_controller