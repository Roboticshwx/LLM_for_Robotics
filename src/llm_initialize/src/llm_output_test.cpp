#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "communication_interfaces/msg/sequence.hpp"
#include "communication_interfaces/msg/targ_x.hpp"
#include "communication_interfaces/msg/oper_x.hpp"


using namespace std::chrono_literals;

class TargetPublisher : public rclcpp::Node
{
public:
  TargetPublisher()
  : Node("target_publisher") {
    // 创建publisher, topic名称为"sequence_topic"
    publisher_ = this->create_publisher<communication_interfaces::msg::Sequence>("sequence_topic", 10);

    std::this_thread::sleep_for(1000ms); // 确保节点已经初始化

    auto message = communication_interfaces::msg::Sequence();

    // 填充 targ_1
    auto targ_1 = communication_interfaces::msg::TargX();
    targ_1.target_x = "A";

    message.targs.push_back(targ_1);

    // targ_2
    auto targ_2 = communication_interfaces::msg::TargX();
    targ_2.target_x = "B";
    
    message.targs.push_back(targ_2);

    publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "Sequence of Robot Operation Publisher!!");

    rclcpp::shutdown();
  }
private:
  rclcpp::Publisher<communication_interfaces::msg::Sequence>::SharedPtr publisher_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TargetPublisher>(); 

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

