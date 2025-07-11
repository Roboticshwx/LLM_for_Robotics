#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"


#include "communication_interfaces/msg/sequence.hpp"

using std::placeholders::_1;

class TargetSubscriber : public rclcpp::Node
{
public:
  TargetSubscriber()
  : Node("target_subscriber")
  {
    subscription_ = this->create_subscription<communication_interfaces::msg::Sequence>(   
      "sequence_topic", 10, std::bind(&TargetSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const communication_interfaces::msg::Sequence::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received message of robot operation sequence.");

    // 遍历所有targ_x
    for (size_t i = 0; i < msg->targs.size(); ++i) {
      const auto& targ = msg->targs[i];

      // 机械臂移动到该目标点
      RCLCPP_INFO(this->get_logger(), "=== 目标点 %zu: %s ===", i + 1, targ.target_x.c_str());

      // 开始遍历操作
      RCLCPP_INFO(this->get_logger(), "  包含 %zu 个操作:", targ.operations.size());

      for (size_t j = 0; j < targ.operations.size(); ++j) {
        const auto& oper = targ.operations[j];

        if (!oper.command.empty()) {
          RCLCPP_INFO(this->get_logger(), "Oper[%zu]: 执行命令 '%s'", j + 1, oper.command.c_str());
          continue;
        }
        
        // 根据类型输出不同维度
        if (oper.type == 0) {
          RCLCPP_INFO(this->get_logger(), "Oper[%zu]: 移液枪 → [%.2f, %.2f]", j + 1, oper.data[0], oper.data[1]);
        } else if (oper.type == 1) {
          RCLCPP_INFO(this->get_logger(), "Oper[%zu]: 旋涂仪 → [%.2f, %.2f, %.2f]", j + 1, oper.data[0], oper.data[1], oper.data[2]);
        } else {
          RCLCPP_WARN(this->get_logger(), "Oper[%zu]: 未知设备 %d (数据长度=%zu)", j + 1, oper.type, oper.data.size());
        }
      }
    }
    RCLCPP_INFO(this->get_logger(), "=== 消息解析完成 ===\n");
  }
  rclcpp::Subscription<communication_interfaces::msg::Sequence>::SharedPtr subscription_; 
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TargetSubscriber>());
  rclcpp::shutdown();
  return 0;
}