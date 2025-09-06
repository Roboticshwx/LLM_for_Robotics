#include <ik_impedance_controller/default_robot_behavior_utils.hpp>
#include <ik_impedance_controller/ik_impedance_controller.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <utility>
#include <Eigen/Dense>



#include <chrono>

using namespace std::chrono_literals;
using Vector7d = Eigen::Matrix<double, 7, 1>;

namespace ik_impedance_controller {


// 配置控制器输入接口
controller_interface::InterfaceConfiguration
IkImpedanceController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

// 配置机械臂的状态读取接口
controller_interface::InterfaceConfiguration
IkImpedanceController::state_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_pose_->get_state_interface_names();
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/position");
  }
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/velocity");
  }
  for (int i = 1; i <= num_joints_; ++i) {
    config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
  }
  for (const auto& franka_robot_model_name : franka_robot_model_->get_state_interface_names()) {
    config.names.push_back(franka_robot_model_name);  // 动力学模型所需要的状态接口
  }

  return config;
}

// 实时采集阻抗控制所需要的关节位置，速度和力矩
void IkImpedanceController::update_joint_states() {
  for (auto i = 0; i < num_joints_; ++i) {
    // 这里还需要进一步优化
    const auto& position_interface = state_interfaces_.at(16 + i);
    const auto& velocity_interface = state_interfaces_.at(23 + i);
    const auto& effort_interface = state_interfaces_.at(30 + i);
    joint_positions_current_[i] = position_interface.get_value();
    joint_velocities_current_[i] = velocity_interface.get_value();
    joint_efforts_current_[i] = effort_interface.get_value();
  }
}


std::shared_ptr<moveit_msgs::srv::GetPositionIK::Request>
IkImpedanceController::create_ik_service_request(
    const Eigen::Vector3d& position,
    const Eigen::Quaterniond& orientation,
    const std::vector<double>& joint_positions_current,
    const std::vector<double>& joint_velocities_current,
    const std::vector<double>& joint_efforts_current) {

  // 创建服务请求对象
  auto service_request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();

  service_request->ik_request.group_name = arm_id_ + "_arm";
  service_request->ik_request.pose_stamped.header.frame_id = arm_id_ + "_link0";
  service_request->ik_request.pose_stamped.pose.position.x = position.x();
  service_request->ik_request.pose_stamped.pose.position.y = position.y();
  service_request->ik_request.pose_stamped.pose.position.z = position.z();
  service_request->ik_request.pose_stamped.pose.orientation.x = orientation.x();
  service_request->ik_request.pose_stamped.pose.orientation.y = orientation.y();
  service_request->ik_request.pose_stamped.pose.orientation.z = orientation.z();
  service_request->ik_request.pose_stamped.pose.orientation.w = orientation.w();
  service_request->ik_request.robot_state.joint_state.name = {
      arm_id_ + "_joint1", arm_id_ + "_joint2", arm_id_ + "_joint3", arm_id_ + "_joint4",
      arm_id_ + "_joint5", arm_id_ + "_joint6", arm_id_ + "_joint7"};
  service_request->ik_request.robot_state.joint_state.position = joint_positions_current;
  service_request->ik_request.robot_state.joint_state.velocity = joint_velocities_current;
  service_request->ik_request.robot_state.joint_state.effort = joint_efforts_current;

  // If Franka Hand is not connected, the following line should be commented out.
  service_request->ik_request.ik_link_name = arm_id_ + "_hand_tcp";
  return service_request;
}


std::pair<Eigen::Vector3d, Eigen::Quaterniond> 
IkImpedanceController::compute_new_position(const Vector7d& target_position_) {

  // 提取前三个元素作为位置 (x, y, z)
  Eigen::Vector3d position_d = target_position_.head<3>();

  // 提取后四个元素构造四元数 (w, x, y, z)
  Eigen::Quaterniond orientation_d(
    target_position_[3],  // w
    target_position_[4],  // x
    target_position_[5],  // y
    target_position_[6]   // z
  );

  // 返回 pair，直接构造即可
  return std::make_pair(position_d, orientation_d);
}


Vector7d IkImpedanceController::compute_torque_command(
    Vector7d& joint_positions_desired,
    Vector7d& joint_positions_current,
    Vector7d& joint_velocities_current) {
  std::array<double, 7> coriolis_array = franka_robot_model_->getCoriolisForceVector();
  Vector7d coriolis(coriolis_array.data());

  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * joint_velocities_current; // 低通滤波器

  Vector7d q_error = joint_positions_desired - joint_positions_current;   // 当前关节误差

  Vector7d tau_d_calculated =
      k_gains_.cwiseProduct(q_error) - d_gains_.cwiseProduct(dq_filtered_);
      //+ coriolis;

  return tau_d_calculated;
}


// 生命周期回调函数
controller_interface::return_type IkImpedanceController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  if (initialization_flag_) {
    std::tie(orientation_, position_) =
        franka_cartesian_pose_->getInitialOrientationAndTranslation();
    initialization_flag_ = false;
  }

  std::lock_guard<std::mutex> lock(ik_mutex_);

  update_joint_states();
  
  if (ik_flag_) {

    RCLCPP_INFO(this->get_node()->get_logger(), "Moving to the target point~~~");

    Eigen::Vector3d new_position = compute_new_position(target_position_).first;
    Eigen::Quaterniond new_orientation = compute_new_position(target_position_).second;

    // 构造请求对象, 注意这里是service client
    auto service_request = create_ik_service_request(
      new_position, 
      new_orientation,
      joint_positions_current_,
      joint_velocities_current_, 
      joint_efforts_current_
    );

    // 使用ROS2 Future定义响应回调类型
    using ServiceResponseFuture = rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedFuture;

    // 设置异步响应回调
    auto response_received_callback = [&](ServiceResponseFuture future) {
      const auto& response = future.get();
      if (response->error_code.val == response->error_code.SUCCESS) {
        joint_positions_desired_ = response->solution.joint_state.position; // 解算成功,返回解算结果
      } else {
        RCLCPP_INFO(this->get_node()->get_logger(), "Waiting for the Inverse Kinematics Solution~~~");
      }
    };
    
    // 异步发送请求
    auto result_future_ = compute_ik_client_->async_send_request(service_request, response_received_callback);

    if (joint_positions_desired_.empty()) {
      this->get_node()->set_parameter({"process_finished", false});

      return controller_interface::return_type::OK;
    }

    // action server被激活, 初始化运动生成器
    if (motion_flag_ == 0) {
      motion_generator_ = std::make_unique<MotionGenerator>(
        0.1,
        Eigen::Map<Vector7d>(joint_positions_current_.data()),
        Eigen::Map<Vector7d>(joint_positions_desired_.data())
      );
      start_time_ = this->get_node()->now();  // 重置时间戳

      motion_flag_ = 1; 
    }

    Vector7d joint_positions_current_eigen(joint_positions_current_.data());
    Vector7d joint_velocities_current_eigen(joint_velocities_current_.data());

    auto trajectory_time = this->get_node()->now() - start_time_;
    auto motion_generator_output = motion_generator_->getDesiredJointPositions(trajectory_time);

    Vector7d q_desired = motion_generator_output.first;  // motion generator插值
    bool finished = motion_generator_output.second;

    if (not finished) {
      for (int i = 0; i < num_joints_; i++) {
        auto tau_d_calculated = compute_torque_command(
          q_desired, 
          joint_positions_current_eigen, 
          joint_velocities_current_eigen
        );
        command_interfaces_[i].set_value(tau_d_calculated(i));
      }
    } else {  // 运动完成

      motion_flag_ = 0;
      ik_flag_ = 0; 

      RCLCPP_INFO(this->get_node()->get_logger(), "Finished~~~");
      this->get_node()->set_parameter({"process_finished", true});
    }
  } else {
    // 控制器没有激活, 则将所有关节力矩设置为0, 等待action client发送坐标
    for (auto& command_interface : command_interfaces_) {
      command_interface.set_value(0);
    }
  }
  return controller_interface::return_type::OK;
}

CallbackReturn IkImpedanceController::on_init() {
  franka_cartesian_pose_ =
      std::make_unique<franka_semantic_components::FrankaCartesianPoseInterface>(
          franka_semantic_components::FrankaCartesianPoseInterface(k_elbow_activated_));

  return CallbackReturn::SUCCESS;
}

bool IkImpedanceController::assign_parameters() {
  arm_id_ = this->get_node()->get_parameter("arm_id").as_string();
  auto k_gains = this->get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = this->get_node()->get_parameter("d_gains").as_double_array();
  if (k_gains.empty()) {
    RCLCPP_FATAL(this->get_node()->get_logger(), "k_gains parameter not set");
    return false;
  }
  if (k_gains.size() != static_cast<uint>(num_joints_)) {
    RCLCPP_FATAL(this->get_node()->get_logger(), "k_gains should be of size %d but is of size %ld",
                  num_joints_, k_gains.size());
    return false;
  }
  if (d_gains.empty()) {
    RCLCPP_FATAL(this->get_node()->get_logger(), "d_gains parameter not set");
    return false;
  }
  if (d_gains.size() != static_cast<uint>(num_joints_)) {
    RCLCPP_FATAL(this->get_node()->get_logger(), "d_gains should be of size %d but is of size %ld",
                  num_joints_, d_gains.size());
    return false;
  }
  for (int i = 0; i < num_joints_; ++i) {
    d_gains_(i) = d_gains.at(i);
    k_gains_(i) = k_gains.at(i);
  }
  return true;
}

CallbackReturn IkImpedanceController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  franka_robot_model_ = std::make_unique<franka_semantic_components::FrankaRobotModel>(
      franka_semantic_components::FrankaRobotModel(arm_id_ + "/" + k_robot_model_interface_name,
                                                    arm_id_ + "/" + k_robot_state_interface_name));
  // 创建客户端
  auto collision_client = this->get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "/service_server/set_full_collision_behavior");
  compute_ik_client_ = this->get_node()->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

  // service client的服务等待逻辑
  while (!compute_ik_client_->wait_for_service(1s) || !collision_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_node()->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(this->get_node()->get_logger(), "service not available, waiting again...");
  }

  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();
  auto future_result = collision_client->async_send_request(request);

  auto success = future_result.get();

  if (!success->success) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(this->get_node()->get_logger(), "Default collision behavior set.");
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn IkImpedanceController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  initialization_flag_ = true;

  dq_filtered_.setZero();

  update_joint_states();

  motion_flag_ = 0;  // motion_generator初始化的标志位
  ik_flag_ = 0;

  target_position_.setZero();

  joint_positions_desired_.reserve(num_joints_);
  joint_positions_current_.reserve(num_joints_);
  joint_velocities_current_.reserve(num_joints_);
  joint_efforts_current_.reserve(num_joints_);

  franka_cartesian_pose_->assign_loaned_state_interfaces(state_interfaces_);
  franka_robot_model_->assign_loaned_state_interfaces(state_interfaces_);

  // 初始化action server
  using namespace std::placeholders;

  action_server_ = rclcpp_action::create_server<Target>(
    this->get_node(),  // 节点指针
    "ik_controller",  // action server名称
    // 绑定action server的回调函数
    std::bind(&IkImpedanceController::handle_goal, this, _1, _2),
    std::bind(&IkImpedanceController::handle_cancel, this, _1),
    std::bind(&IkImpedanceController::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_node()->get_logger(), "Action server 'ik_controller' initialized!!!");

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn IkImpedanceController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_pose_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

// action server 回调函数handle_goal()
rclcpp_action::GoalResponse IkImpedanceController::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const Target::Goal> goal)
{
  // 检查目标点是否为七维向量
  if (goal->target_point.size() != 7) {
    RCLCPP_ERROR(this->get_node()->get_logger(), "Invalid target point size!!!");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_node()->get_logger(), "Received target point request with 7 dimenstion.");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

// action server 回调函数handle_cancel()
rclcpp_action::CancelResponse IkImpedanceController::handle_cancel(
  const std::shared_ptr<GoalHandleTarget> goal_handle)
{
  RCLCPP_INFO(this->get_node()->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

// action server 回调函数handle_accepted()
void IkImpedanceController::handle_accepted(
  const std::shared_ptr<GoalHandleTarget> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&IkImpedanceController::execute, this, _1), goal_handle}.detach();
}

// action server 执行函数execute()
void IkImpedanceController::execute(
  const std::shared_ptr<GoalHandleTarget> goal_handle
)
{
  RCLCPP_INFO(this->get_node()->get_logger(), "Executing goal");

  // 获取七维向量
  const auto goal = goal_handle->get_goal();

	auto feedback = std::make_shared<Target::Feedback>();
  auto result = std::make_shared<Target::Result>();

  std::unique_lock<std::mutex> lock(ik_mutex_);

  for (int i = 0; i < 7; i++) {
    target_position_(i) = goal->target_point[i];
  }

  ik_flag_ = 1; // 激活控制器

  lock.unlock();

  rclcpp::Rate loop_rate(10); // 反馈频率为10Hz

  // 等待运动完成
  while (rclcpp::ok()) {

    // 更新反馈信息
    feedback->current_state = "Moving";
    goal_handle->publish_feedback(feedback);

    // 检查是否被取消
    if (goal_handle->is_canceling()) {
      result->success = false;
      result->status = "Canceled";
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_node()->get_logger(), "Goal canceled");
      return;
    }

    this->get_node()->get_parameter("process_finished", process_finished_);

    if (process_finished_) {
      RCLCPP_INFO(this->get_node()->get_logger(), "IK Move Process finished~~~");
    
      break;
    }

    loop_rate.sleep();
  }

  result->success = true;
  result->status = "Goal reached successfully";
  this->get_node()->set_parameter({"process_finished", false});

  // “做好清理，藏好自己”
  joint_positions_desired_.clear();

  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_node()->get_logger(), "Goal succeeded");
}

}  // namespace ik_impedance_controller
#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(ik_impedance_controller::IkImpedanceController,controller_interface::ControllerInterface)
