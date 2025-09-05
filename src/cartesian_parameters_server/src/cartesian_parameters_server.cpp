#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <unordered_map>

#include "communication_interfaces/srv/cartesian.hpp"

void handle_cartesian_request(
const std::shared_ptr<communication_interfaces::srv::Cartesian::Request> request,
std::shared_ptr<communication_interfaces::srv::Cartesian::Response> response)
{
	// pre-define map for the cartesian parameters
	static const std::unordered_map<std::string, std::array<double, 4>> vector_map {
		{"cart_1", {0.05, 0.00, 0.00, 3}},
		{"cart_2", {0.00, 0.05, 0.00, 3}},
    {"cart_3", {0.00, 0.00, 0.05, 3}},
		{"cart_4", {0.05, 0.05, 0.05, 3}},
	};

	const std::string& input_key = request->cartesian_key;

	if (vector_map.count(input_key)) {	// 检查输入是否符合要求
		// 将目标点与对应坐标进行一一映射
		const auto& cart_vector = vector_map.at(input_key);
		std::copy(cart_vector.begin(), cart_vector.end(), response->cartesian_vector.begin());

		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
			"Valid request: [%s] -> Vector: [%f, %f, %f, %f]",
			input_key.c_str(),
			cart_vector[0], cart_vector[1], cart_vector[2], cart_vector[3]);
	} else {
		// 如果输入不符合要求, 返回空的目标点向量
		std::fill(response->cartesian_vector.begin(), response->cartesian_vector.end(), 0);

		RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
			"Invalid request: [%s] (Expected single character A-F)",
			input_key.c_str());
	}
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("cartesian_parameters_server"); // node named "cartesian_parameters_server"

	rclcpp::Service<communication_interfaces::srv::Cartesian>::SharedPtr service = node->create_service<communication_interfaces::srv::Cartesian>("move_cartesian", &handle_cartesian_request);	// service named "move_cartesian"

	RCLCPP_INFO(node->get_logger(), "Ready to receive cartesian parameters requests.");

	rclcpp::spin(node);
	rclcpp::shutdown();
}