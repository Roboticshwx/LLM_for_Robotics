#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <unordered_map>

#include "communication_interfaces/srv/target_point.hpp"

void handle_target_request(
const std::shared_ptr<communication_interfaces::srv::TargetPoint::Request> request,
std::shared_ptr<communication_interfaces::srv::TargetPoint::Response> response)
{
	// pre-define map for the target point
	static const std::unordered_map<std::string, std::array<double, 7>> vector_map {
		{"A", {0.29402958, -0.32655160, 0.38956459, 0.02623328, 0.97555703, -0.21804896, -0.00819949}},
		{"B", {0.44246598, 0.26825204, 0.30625534, -0.02599562, 0.99931091, 0.02593865, 0.00539561}},
	};

	const std::string& input_key = request->target_point;

	if (input_key.length() == 1 && vector_map.count(input_key)) {	// 检查输入是否符合要求
		// 将目标点与对应坐标进行一一映射
		const auto& target_vector = vector_map.at(input_key);
		std::copy(target_vector.begin(), target_vector.end(), response->target_point_vector.begin());

		RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
			"Valid request: [%s] -> Vector: [%f, %f, %f, %f, %f, %f, %f]",
			input_key.c_str(),
			target_vector[0], target_vector[1], target_vector[2],
			target_vector[3], target_vector[4], target_vector[5], target_vector[6]);
	} else {
		// 如果输入不符合要求, 返回空的目标点向量
		std::fill(response->target_point_vector.begin(), response->target_point_vector.end(), 0);

		RCLCPP_WARN(rclcpp::get_logger("rclcpp"), 
			"Invalid request: [%s] (Expected single character A-F)",
			input_key.c_str());
	}
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	
	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("target_point_server");

	rclcpp::Service<communication_interfaces::srv::TargetPoint>::SharedPtr service = node->create_service<communication_interfaces::srv::TargetPoint>("get_target_point", &handle_target_request);

	RCLCPP_INFO(node->get_logger(), "Ready to receive target point requests.");

	rclcpp::spin(node);
	rclcpp::shutdown();
}