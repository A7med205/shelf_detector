#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/rclcpp.hpp"

class ParameterClientNode : public rclcpp::Node {
public:
  ParameterClientNode() : Node("parameter_client_node") {
    // Create a client to the SetParameters service
    param_client_ = this->create_client<rcl_interfaces::srv::SetParameters>(
        "/local_costmap/local_costmap/set_parameters");

    // Wait for the service to be available
    while (!param_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_INFO(this->get_logger(), "Waiting for parameter server...");
    }

    // Create a parameter request
    auto request =
        std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request->parameters.push_back(
        rclcpp::Parameter("robot_radius", 0.3).to_parameter_msg());

    auto future = param_client_->async_send_request(
        request, std::bind(&ParameterClientNode::future_response, this,
                           std::placeholders::_1));
  }

private:
  void future_response(
      rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
    auto status = future.wait_for(std::chrono::seconds(1));

    if (status == std::future_status::ready) {
      auto response = future.get();
      for (const auto &result : response->results) {
        if (result.successful) {
          RCLCPP_INFO(this->get_logger(), "Parameter set successfully.");
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to set parameter: %s",
                      result.reason.c_str());
        }
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr param_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ParameterClientNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
