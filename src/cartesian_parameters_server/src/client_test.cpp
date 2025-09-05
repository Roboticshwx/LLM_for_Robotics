#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <chrono>
#include <cstdlib>
#include <string>

#include "communication_interfaces/srv/cartesian.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 2) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Usage: cartesian_client cart_x");
    return 1;
  }

  // 创建Node
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("cartesian_parameters_test");

  // 创建客户端
  rclcpp::Client<communication_interfaces::srv::Cartesian>::SharedPtr client = node->create_client<communication_interfaces::srv::Cartesian>("move_cartesian");

  // 构造请求对象，并将命令行参数赋值给目标点名称
  auto request = std::make_shared<communication_interfaces::srv::Cartesian::Request>();
  request->cartesian_key = argv[1];  // 从命令行参数获取目标点名称

  // 等待服务端上线
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {  // 检查ROS2是否正常运行
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service.");
      return 0;
    }
    RCLCPP_INFO(node->get_logger(), "Service not available, waiting...");
  }

  auto result_future = client->async_send_request(request);

  // 异步等待响应
  if (rclcpp::spin_until_future_complete(node, result_future) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto response = result_future.get();
    const auto& cart_vec = response->cartesian_vector;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
      "cartesian vector: [%f, %f, %f, %f]",
      cart_vec[0], cart_vec[1], cart_vec[2], cart_vec[3]); 
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service move_cartesian!!!");
  }

  rclcpp::shutdown();
  return 0;
}


