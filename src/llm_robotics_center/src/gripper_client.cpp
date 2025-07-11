#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "franka_msgs/action/move.hpp"


namespace franka_gripper_client
{
class GripperActionClient : public rclcpp::Node
{
public:
  using Grasp = franka_msgs::action::Move; // 定义action类型
  using GoalHandleGrasp = rclcpp_action::ClientGoalHandle<Grasp>;

  explicit GripperActionClient(const rclcpp::NodeOptions & options)
  : Node("llm_robotics_control_center", options)  // 调用构造函数创建节点
  {
    // this指针指向当前节点
    this->client_ptr_ = rclcpp_action::create_client<Grasp>(
      this, "/fr3_gripper/move"); // 创建action client, 名称需与action server一致

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&GripperActionClient::send_goal, this));
  }

  // 向server发送的目标函数
  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    // 阻塞式等待action server上线
    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    // 创建目标请求消息
    auto goal_msg = Grasp::Goal();
    // 这个是用于测试的目标点的七维向量
    goal_msg.width = 0.04;
    goal_msg.speed = 0.01;
    

    // 在终端中表示已经发送目标点的请求
    RCLCPP_INFO(this->get_logger(), "Sending goal");

    // 后续的回调函数绑定到目标点的发送之后的流程中
    auto send_goal_options = rclcpp_action::Client<Grasp>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&GripperActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&GripperActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&GripperActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Grasp>::SharedPtr client_ptr_;  // action client指针
  rclcpp::TimerBase::SharedPtr timer_;  // 定时器指针

  // 目标响应回调函数
  // 响应action server的handle_goal(), 将结果反馈到终端上
  void goal_response_callback(const GoalHandleGrasp::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Grasp Goal accepted by server, waiting for result");
    }
  }

  // 反馈回调函数
  void feedback_callback(
    GoalHandleGrasp::SharedPtr,
    const std::shared_ptr<const Grasp::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), 
      "Current width: %.4fm",
      feedback->current_width);
  }

  // 结果回调函数
  void result_callback(const GoalHandleGrasp::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Robot action completed~");
        break;
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
    rclcpp::shutdown();
  }
};  // class GripperActionClient

}  // namespace franka_gripper_client

RCLCPP_COMPONENTS_REGISTER_NODE(franka_gripper_client::GripperActionClient)