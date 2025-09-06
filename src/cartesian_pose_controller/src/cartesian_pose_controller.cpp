#include <cartesian_pose_controller/cartesian_pose_controller.hpp>
#include <cartesian_pose_controller/default_robot_behavior_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <chrono>

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

  if (initialization_flag_) {
    // Get initial orientation and translation
    std::tie(orientation_, position_) = franka_cartesian_pose_->getInitialOrientationAndTranslation();
    initialization_flag_ = false;
  }
  
  if (cartesian_flag_) {
    elapsed_time_ = elapsed_time_ + trajectory_period_; // 跟踪轨迹运行时间
    
    new_position = position_;
    new_orientation = orientation_;

    if (elapsed_time_ < motion_duration_) {
      RCLCPP_INFO(this->get_node()->get_logger(), "Tested~~~");

      double factor = (1.0 - std::cos(M_PI * elapsed_time_ / motion_duration_)) / 2.0;

      new_position(0) = position_(0) + x_displacement_ * factor; // x轴方向运动
      new_position(1) = position_(1) + y_displacement_ * factor; // y轴方向运动
      new_position(2) = position_(2) + z_displacement_ * factor; // z轴方向运动
    } else {
      new_position(0) = position_(0) + x_displacement_;
      new_position(1) = position_(1) + y_displacement_;
      new_position(2) = position_(2) + z_displacement_;

      // 关闭控制器
      cartesian_flag_ = false;

      // 更新末端位置到上一次的终点
      position_ = new_position;
      orientation_ = new_orientation;

      this->get_node()->set_parameter({"cartesian_finished", true});

      RCLCPP_INFO(this->get_node()->get_logger(), "Finished~~~");
    }

    if (franka_cartesian_pose_->setCommand(new_orientation, new_position)) {
      return controller_interface::return_type::OK;
    } else {
      RCLCPP_FATAL(get_node()->get_logger(),
                    "Set command failed. Did you activate the elbow command interface?");
      return controller_interface::return_type::ERROR;
    }
  }
  return controller_interface::return_type::OK;
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
  cartesian_flag_ = false;
  franka_cartesian_pose_->assign_loaned_command_interfaces(command_interfaces_);
  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);


  using namespace std::placeholders;

  action_server_ = rclcpp_action::create_server<Cartesian>(
    this->get_node(),  // 节点指针
    "cartesian_pose_controller",  // action server名称
    std::bind(&CartesianPoseController::handle_goal, this, _1, _2),
    std::bind(&CartesianPoseController::handle_cancel, this, _1),
    std::bind(&CartesianPoseController::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_node()->get_logger(), "Action server 'cartesian_pose_controller' initialized!!!");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPoseController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

// action server 回调函数handle_goal()
rclcpp_action::GoalResponse CartesianPoseController::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Cartesian::Goal> goal)
{
  // 检查目标点是否为四维向量
  if (goal->cartesian_move.size() != 4) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Invalid cartesian vector size!!!");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_node()->get_logger(), "Received cartesian move request with 4 dimenstion.");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// action server 回调函数handle_cancel()
rclcpp_action::CancelResponse CartesianPoseController::handle_cancel(
  const std::shared_ptr<GoalHandleCartesian> goal_handle)
{
  RCLCPP_INFO(this->get_node()->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

// action server 回调函数handle_accepted()
void CartesianPoseController::handle_accepted(
  const std::shared_ptr<GoalHandleCartesian> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&CartesianPoseController::execute, this, _1), goal_handle}.detach();
}

// action server 执行函数execute()
void CartesianPoseController::execute(
  const std::shared_ptr<GoalHandleCartesian> goal_handle
)
{
  RCLCPP_INFO(this->get_node()->get_logger(), "Executing goal");

  // 获取四维向量
  const auto goal = goal_handle->get_goal();

  auto feedback = std::make_shared<Cartesian::Feedback>();
  auto result = std::make_shared<Cartesian::Result>();

  for (int i = 0; i < 4; ++i) {
    cartesian_move_(i) = goal->cartesian_move[i];
  }

  // 设置运动参数
  elapsed_time_ = 0.0;  // 重置计数器
  x_displacement_ = cartesian_move_(0); // x位移
  y_displacement_ = cartesian_move_(1); // y位移
  z_displacement_ = cartesian_move_(2); // z位移
  motion_duration_ = cartesian_move_(3);  // 运动时间

  RCLCPP_INFO(this->get_node()->get_logger(), "Set Cartesian Parameters~~~");

  // 启动控制器
  cartesian_flag_ = true;

  rclcpp::Rate loop_rate(10); // 1kHz feedback

  // 等待运动完成
  while (rclcpp::ok()) {
    feedback->current_state = "Moving";
    goal_handle->publish_feedback(feedback);

    // 检查是否被取消
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->status = "Goal canceled.";
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_node()->get_logger(), "Goal canceled");
      cartesian_flag_ = false;
      return;
    }

    this->get_node()->get_parameter("cartesian_finished", cartesian_finished_);

    if (cartesian_finished_) {
      RCLCPP_INFO(this->get_node()->get_logger(), "Cartesian movement finished~~~");
      break;
    }
    loop_rate.sleep();
  }

  result->success = true;
  result->status = "Goal reached successfully.";
  this->get_node()->set_parameter({"cartesian_finished", false});

  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_node()->get_logger(), "Goal succeeded");
}



}  // namespace cartesian_pose_controller
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cartesian_pose_controller::CartesianPoseController,
                        controller_interface::ControllerInterface)