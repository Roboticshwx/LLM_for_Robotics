#pragma once

#include <Eigen/Dense>
#include <string>
#include <utility>

#include <controller_interface/controller_interface.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>
#include "franka_semantic_components/franka_cartesian_pose_interface.hpp"
#include "franka_semantic_components/franka_robot_model.hpp"

#include "motion_generator.hpp"

#include <functional>
#include <memory>
#include <thread>

#include "communication_interfaces/action/target.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"


using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace ik_impedance_controller {


class IkImpedanceController : public controller_interface::ControllerInterface {
public:

  using Vector7d = Eigen::Matrix<double, 7, 1>;

  IkImpedanceController() = default;
  // 接收NodeOptions参数的构造函数

  // 定义action类型
  using Target = communication_interfaces::action::Target; 
  using GoalHandleTarget = rclcpp_action::ServerGoalHandle<Target>;

  // 控制器接口
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration()
      const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration()
      const override;
  controller_interface::return_type update(const rclcpp::Time& time,
                                            const rclcpp::Duration& period) override;

  // ros2 control 生命周期回调函数
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  // action server回调函数
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const Target::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTarget> goal_handle);
  void handle_accepted(
    const std::shared_ptr<GoalHandleTarget> goal_handle);  // 使用GoalHandleTarget
  void execute(const std::shared_ptr<GoalHandleTarget> goal_handle);

private:
  // action server的实例
  rclcpp_action::Server<Target>::SharedPtr action_server_;
  
  // std::shared_ptr<rclcpp::Node> node_;

  void update_joint_states();

  std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request> create_ik_service_request(
      const Eigen::Vector3d& new_position,
      const Eigen::Quaterniond& new_orientation,
      const std::vector<double>& joint_positions_desired,
      const std::vector<double>& joint_positions_current,
      const std::vector<double>& joint_efforts_current);

  Vector7d compute_torque_command(Vector7d& joint_positions_desired,
                                  Vector7d& joint_positions_current,
                                  Vector7d& joint_velocities_current
  );

  std::pair<Eigen::Vector3d, Eigen::Quaterniond> compute_new_position(const Vector7d& target_position_);
  
  bool assign_parameters();

  std::unique_ptr<franka_semantic_components::FrankaCartesianPoseInterface> franka_cartesian_pose_;

  Eigen::Quaterniond orientation_;
  Eigen::Vector3d position_;

  Vector7d target_position_; // 目标位置

  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_client_;

  double trajectory_period_{0.001};
  const bool k_elbow_activated_{false};
  bool initialization_flag_{true};

  std::string arm_id_;

  double elapsed_time_{0.0};
  std::unique_ptr<franka_semantic_components::FrankaRobotModel> franka_robot_model_;

  const std::string k_robot_state_interface_name{"robot_state"};
  const std::string k_robot_model_interface_name{"robot_model"};

  Vector7d dq_filtered_;
  Vector7d k_gains_;
  Vector7d d_gains_;
  int num_joints_{7};

  std::unique_ptr<MotionGenerator> motion_generator_;   // 轨迹插值器
  rclcpp::Time start_time_;                             // 轨迹开始时间

  bool motion_flag_;           // 执行逆解并且运动的标志位
  bool process_finished_{false}; // 进程是否完成的标志位
  bool ik_flag_;


  std::vector<double> joint_positions_desired_;
  std::vector<double> previous_joint_positions_desired_;
  std::vector<double> joint_positions_current_{0, 0, 0, 0, 0, 0, 0};
  std::vector<double> joint_velocities_current_{0, 0, 0, 0, 0, 0, 0};
  std::vector<double> joint_efforts_current_{0, 0, 0, 0, 0, 0, 0};


  std::mutex ik_mutex_;  // 用于保护逆解计算的互斥锁

};
}  // namespace ik_impedance_controller

