#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>
#include <chrono>
#include <cstdlib>
#include <queue>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "communication_interfaces/msg/sequence.hpp"
#include "communication_interfaces/msg/targ_x.hpp"
#include "communication_interfaces/msg/oper_x.hpp"

#include "communication_interfaces/action/target.hpp"
#include "communication_interfaces/srv/target_point.hpp"
#include "communication_interfaces/srv/grasp.hpp"
#include "franka_msgs/action/move.hpp"

namespace llm_robotics_center
{

enum class ActionType {
  MOVE_ARM,                  // 移动机械到目标点
  GRASP_OBJECT,              // 夹爪动作
  DEVICE_0,                  // 处理移液枪
  DEVICE_1,                  // 处理旋涂仪
  // CARTESIAN_MOVE,           // 笛卡尔空间移动
  UNKNOWN_OPERATION          // 未知动作
};

struct RobotAction {
  ActionType type;           // 动作类型
  std::string name;          // 用于 MOVE_ARM (target_x) 和 GRASP_OBJECT (command)
  std::vector<double> data;  // 用于与设备通信的oper.data
};

class LLMRoboticsCenter : public rclcpp::Node
{
public:
  // action server的消息类型
  using Target = communication_interfaces::action::Target;
  using Grasp = franka_msgs::action::Move;

  using GoalHandleTarget = rclcpp_action::ClientGoalHandle<Target>;
  using GoalHandleGrasp = rclcpp_action::ClientGoalHandle<Grasp>;

  // service server的消息类型
  using TargetPointService = communication_interfaces::srv::TargetPoint;
  using GraspService = communication_interfaces::srv::Grasp;

  // subscriber的消息类型
  using SequenceMsg = communication_interfaces::msg::Sequence;
  using TargXMsg = communication_interfaces::msg::TargX;
  using OperXMsg = communication_interfaces::msg::OperX;

  explicit LLMRoboticsCenter(const rclcpp::NodeOptions & options)
  : Node("llm_robotics_control_center", options)  // 调用构造函数创建节点
  {
    // 创建action client和service client
    this->ik_service_client_ = this->create_client<TargetPointService>("get_target_point");
    this->ik_action_client_ = rclcpp_action::create_client<Target>(this, "ik_controller");

    this->grasp_service_client_ = this->create_client<GraspService>("move_gripper");
    this->grasp_action_client_ = rclcpp_action::create_client<Grasp>(this, "/fr3_gripper/move");

    // 创建subscriber，绑定回调函数sequence_callback
    this->sequence_subscription_ = this->create_subscription<SequenceMsg>(
      "sequence_topic", 10, std::bind(&LLMRoboticsCenter::sequence_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Node initialized. Waiting for robot operation sequences~~~~");
  }

private:
  rclcpp_action::Client<Target>::SharedPtr ik_action_client_; 
  rclcpp_action::Client<Grasp>::SharedPtr grasp_action_client_;
  rclcpp::Client<TargetPointService>::SharedPtr ik_service_client_;
  rclcpp::Client<GraspService>::SharedPtr grasp_service_client_;
  rclcpp::Subscription<SequenceMsg>::SharedPtr sequence_subscription_;

  std::array<double, 7> target_point_vector_; // 存储ik service server返回的目标点向量
  std::array<double, 2> grasp_vector_; // 存储grasp service server返回的抓取参数向量

  std::queue<RobotAction> action_sequence_; // 存储机器人动作序列
  bool is_executing_sequence_ = false; // 标记是否正在执行动作序列

  void sequence_callback(const SequenceMsg::SharedPtr msg)
  {
    // 清空缓存
    while (!action_sequence_.empty()) {
      action_sequence_.pop();
    }
    RCLCPP_INFO(this->get_logger(), "Cleared previous action sequence.");


    // 遍历所有目标点
    for (size_t i = 0; i < msg->targs.size(); ++i) {
      const auto& targ = msg->targs[i];

      action_sequence_.push({ActionType::MOVE_ARM, targ.target_x, {}});
      RCLCPP_INFO(this->get_logger(), "  Added MOVE_ARM to '%s'", targ.target_x.c_str()); 

      // 遍历所有操作
      for (size_t j = 0; j < targ.operations.size(); ++j) {
        const auto& oper = targ.operations[j];

        // 字符串的为夹爪操作，否则则是与设备通信
        if (!oper.command.empty()) {
          action_sequence_.push({ActionType::GRASP_OBJECT, oper.command, {}});
          RCLCPP_INFO(this->get_logger(), "  Added GRASP_OBJECT to '%s'", oper.command.c_str());
          continue;
        } else {
          if (oper.type == 0) { // 移液枪
            action_sequence_.push({ActionType::DEVICE_0, "", oper.data});
            RCLCPP_INFO(this->get_logger(), "  Added DEVICE_0 with data [%.2f, %.2f]", oper.data[0], oper.data[1]);
          } else if (oper.type == 1) { // 旋涂仪
            action_sequence_.push({ActionType::DEVICE_1, "", oper.data});
            RCLCPP_INFO(this->get_logger(), "  Added DEVICE_1 with data [%.2f, %.2f, %.2f]", oper.data[0], oper.data[1], oper.data[2]);
          } else {
            action_sequence_.push({ActionType::UNKNOWN_OPERATION, "", {}});
            RCLCPP_WARN(this->get_logger(), "  Added UNKNOWN_OPERATION with type %d", oper.type);
          }
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "Action queue populated with %zu actions.", action_sequence_.size());

    // 启动第一个动作
    is_executing_sequence_ = true;
    execute_next_action();
  }

  // 根据动作队列不断执行下一个动作
  void execute_next_action()
  {
    if (action_sequence_.empty()) {
      RCLCPP_INFO(this->get_logger(), "All actions in sequence completed!");
      rclcpp::shutdown(); // 所有动作执行完毕，可以关闭节点
      return;
    }

    RobotAction next_action = action_sequence_.front();
    action_sequence_.pop(); // 取出并移除队列头部的动作

    switch (next_action.type) {
      case ActionType::MOVE_ARM:
        RCLCPP_INFO(this->get_logger(), "Executing MOVE_ARM: %s", next_action.name.c_str());
        call_ik_service_action(next_action.name);
        break;
      case ActionType::GRASP_OBJECT:
        RCLCPP_INFO(this->get_logger(), "Executing GRASP_OBJECT: %s", next_action.name.c_str());
        call_gripper_service_action(next_action.name);
        break;
      case ActionType::DEVICE_0:
        RCLCPP_INFO(this->get_logger(), "Executing DEVICE_0 (Pipette) with data: [%.2f, %.2f]",
                    next_action.data[0], next_action.data[1]);
        handle_device_0(next_action.data);
        execute_next_action();
        break;
      case ActionType::DEVICE_1:
        RCLCPP_INFO(this->get_logger(), "Executing DEVICE_1 (Spin Coater) with data: [%.2f, %.2f, %.2f]",
                    next_action.data[0], next_action.data[1], next_action.data[2]);
        handle_device_1(next_action.data);
        execute_next_action();
        break;
      case ActionType::UNKNOWN_OPERATION:
        RCLCPP_WARN(this->get_logger(), "Skipping unknown operation: %s", next_action.name.c_str());
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unrecognized action type encountered!");
        break;
    }
  }

  // 获取目标点参数并且通过ik controller移动到目标点
  void call_ik_service_action(const std::string & target_name) {
    using namespace std::chrono_literals;

    auto request = std::make_shared<TargetPointService::Request>();
    request->target_point = target_name;

    while (!ik_service_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "IK Service not available, waiting...");
    }

    // 等待ik action server上线
    if (!this->ik_action_client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "IK Action server not available after waiting");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Requesting target point: %s", target_name.c_str());

    ik_service_client_->async_send_request(request, std::bind(&LLMRoboticsCenter::handle_ik_service_response, this, std::placeholders::_1));
  }

  // 获取抓取参数并且通过gripper执行动作
  void call_gripper_service_action(const std::string & grasp_name) {
    using namespace std::chrono_literals;

    auto request = std::make_shared<GraspService::Request>();
    request->grasp_key = grasp_name;

    while (!grasp_service_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Grasp Service not available, waiting...");
    }

    // 等待grasp action server上线
    if (!this->grasp_action_client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Grasp Action server not available after waiting");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Requesting move parameters: %s", grasp_name.c_str());

    grasp_service_client_->async_send_request(request, std::bind(&LLMRoboticsCenter::handle_grasp_service_response, this, std::placeholders::_1)); 
  }

  // 与ik controller的action server通信
  void handle_ik_service_response(rclcpp::Client<TargetPointService>::SharedFuture future)
  {
    if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) { // 确认 future 已经完成
      auto response = future.get();
      const std::array<double,7> target_point_vector_ = response->target_point_vector; 

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
      "Target vector: [%f, %f, %f, %f, %f, %f, %f]",
      target_point_vector_[0], target_point_vector_[1], target_point_vector_[2],
      target_point_vector_[3], target_point_vector_[4], target_point_vector_[5], target_point_vector_[6]);

      auto goal_msg = Target::Goal();
      goal_msg.target_point = target_point_vector_; // 来自service server的目标点位姿
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
      "Sending Goal to Action Server~~~: [%f, %f, %f, %f, %f, %f, %f]",
      target_point_vector_[0], target_point_vector_[1], target_point_vector_[2],
      target_point_vector_[3], target_point_vector_[4], target_point_vector_[5], target_point_vector_[6]);

      // action的回调函数绑定
      auto send_goal_options = rclcpp_action::Client<Target>::SendGoalOptions();
      send_goal_options.goal_response_callback = std::bind(&LLMRoboticsCenter::ik_goal_response_callback, this, std::placeholders::_1);
      send_goal_options.feedback_callback = std::bind(&LLMRoboticsCenter::ik_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback = std::bind(&LLMRoboticsCenter::ik_result_callback, this, std::placeholders::_1);
      this->ik_action_client_->async_send_goal(goal_msg, send_goal_options);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to get response from get_target_point service or future not ready!");
    }
  }

  // 与gripper的action server通信
  void handle_grasp_service_response(rclcpp::Client<GraspService>::SharedFuture future)
  {
    if (future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) { // 确认 future 已经完成
      auto response = future.get();
      const std::array<double,2> grasp_vector_ = response->grasp_vector; 

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Grasp Parameters Vector: [%f, %f]", grasp_vector_[0], grasp_vector_[1]);

      auto goal_msg = Grasp::Goal();
      goal_msg.width = grasp_vector_[0]; // 来自service server的抓取宽度
      goal_msg.speed = grasp_vector_[1]; // 来自service server的抓取速度

      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sending Goal to Action Server~~~: [%f, %f]", grasp_vector_[0], grasp_vector_[1]);

      // action的回调函数绑定
      auto send_goal_options = rclcpp_action::Client<Grasp>::SendGoalOptions();
      send_goal_options.goal_response_callback = std::bind(&LLMRoboticsCenter::grasp_goal_response_callback, this, std::placeholders::_1);
      send_goal_options.feedback_callback = std::bind(&LLMRoboticsCenter::grasp_feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
      send_goal_options.result_callback = std::bind(&LLMRoboticsCenter::grasp_result_callback, this, std::placeholders::_1);
      this->grasp_action_client_->async_send_goal(goal_msg, send_goal_options);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to get response from move_gripper service or future not ready!");
    }
  }

  // 目标响应回调函数
  // 响应action server的handle_goal(), 将结果反馈到终端上
  void ik_goal_response_callback(const GoalHandleTarget::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void grasp_goal_response_callback(const GoalHandleGrasp::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Grasp Goal accepted by server, waiting for result");
    }
  }

  // 反馈回调函数
  void ik_feedback_callback(GoalHandleTarget::SharedPtr, const std::shared_ptr<const Target::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), 
      "Current state: %s",
      feedback->current_state.c_str());
  }

  void grasp_feedback_callback(GoalHandleGrasp::SharedPtr, const std::shared_ptr<const Grasp::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), 
      "Current width: %.4fm",
      feedback->current_width);
  }

  // 结果回调函数
  void ik_result_callback(const GoalHandleTarget::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Robot action completed~");
        execute_next_action(); // 执行下队列中的下一个动作
        return;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
  }

  void grasp_result_callback(const GoalHandleGrasp::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Gripper action completed~");
        execute_next_action(); // 执行下队列中的下一个动作
        return;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Grasp Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Grasp Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
  }

  // 移液枪回调函数
  void handle_device_0(const std::vector<double>& data) {
    RCLCPP_INFO(this->get_logger(), "Handling liquid handler with data: [%.2f, %.2f]", data[0], data[1]);
    std::this_thread::sleep_for(std::chrono::seconds(8));
  }

  // 旋涂仪回调函数
  void handle_device_1(const std::vector<double>& data) {
    RCLCPP_INFO(this->get_logger(), "Handling spin coater with data: [%.2f, %.2f, %.2f]", data[0], data[1], data[2]);
    std::this_thread::sleep_for(std::chrono::seconds(10));
  }
};  // class LLMRoboticsCenter

}  // namespace llm_robotics_center

RCLCPP_COMPONENTS_REGISTER_NODE(llm_robotics_center::LLMRoboticsCenter)