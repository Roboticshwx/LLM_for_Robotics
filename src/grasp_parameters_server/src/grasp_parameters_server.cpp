#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <unordered_map>

#include "communication_interfaces/srv/grasp.hpp"

void handle_grasp_request(
const std::shared_ptr<communication_interfaces::srv::Grasp::Request> request,
std::shared_ptr<communication_interfaces::srv::Grasp::Response> response)
{
	// pre-define map for the gripper parameters
	static const std::unordered_map<std::string, std::array<double, 2>> vector_map {
		{"grasp_1", {0.01, 0.01}},
		{"grasp_2", {0.04, 0.01}},
    {"grasp_3", {0.03, 0.04}},
	};

	const std::string& input_key = request->grasp_key;

	if (vector_map.count(input_key)) {	// 检查输入是否符合要求
		// 将目标点与对应坐标进行一一映射
		const auto& move_vector = vector_map.at(input_key);
		std::copy(move_vector.begin(), move_vector.end(), response->grasp_vector.begin());

		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
			"Valid request: [%s] -> Vector: [%f, %f]",
			input_key.c_str(),
			move_vector[0], move_vector[1]);
	} else {
		// 如果输入不符合要求, 返回空的目标点向量
		std::fill(response->grasp_vector.begin(), response->grasp_vector.end(), 0);

		RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
			"Invalid request: [%s] (Expected single character A-F)",
			input_key.c_str());
	}
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("grasp_parameters_server");

	rclcpp::Service<communication_interfaces::srv::Grasp>::SharedPtr service = node->create_service<communication_interfaces::srv::Grasp>("move_gripper", &handle_grasp_request);

	RCLCPP_INFO(node->get_logger(), "Ready to receive grasp parameters requests.");

	rclcpp::spin(node);
	rclcpp::shutdown();
}