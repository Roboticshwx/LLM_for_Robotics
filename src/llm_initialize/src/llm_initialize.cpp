#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "communication_interfaces/msg/sequence.hpp"

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
    
    // auto oper_1 = communication_interfaces::msg::OperX();
    // oper_1.type = 0;
    // oper_1.data = {1.0, 2.0};
    // targ_1.operations.push_back(oper_1);

    auto grasp_1 = communication_interfaces::msg::OperX();
    grasp_1.command = "grasp_1";
    targ_1.operations.push_back(grasp_1);

    message.targs.push_back(targ_1);

    // targ_2
    auto targ_2 = communication_interfaces::msg::TargX();
    targ_2.target_x = "B";
    
    auto grasp_2 = communication_interfaces::msg::OperX();
    grasp_2.command = "grasp_2";
    targ_2.operations.push_back(grasp_2);

    message.targs.push_back(targ_2);

    // 填充 targ_1
    auto targ_3 = communication_interfaces::msg::TargX();
    targ_3.target_x = "A";

    auto grasp_3 = communication_interfaces::msg::OperX();
    grasp_3.command = "grasp_1";
    targ_3.operations.push_back(grasp_3);

    message.targs.push_back(targ_3);


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

