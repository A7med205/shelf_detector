#include <nav2_apps/srv/go_to_loading.hpp>
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

class AddTwoIntsClientNode : public rclcpp::Node {
public:
  AddTwoIntsClientNode() : Node("add_two_ints_client") {
    srv_client_ =
        this->create_client<nav2_apps::srv::GoToLoading>("approach_shelf");

    // Wait until the service is available
    while (!srv_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
    }

    // Create a request
    auto request = std::make_shared<nav2_apps::srv::GoToLoading::Request>();
    request->attach_to_shelf = 1;

    // Send the request
    auto result_future = srv_client_->async_send_request(request);

    // Spin until the result is available
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                           result_future) ==
        rclcpp::FutureReturnCode::SUCCESS) {
      auto response = result_future.get();
      RCLCPP_INFO(this->get_logger(), "Result: %d", response->complete);
    } else {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to receive a response from the server.");
    }
  }

private:
  rclcpp::Client<nav2_apps::srv::GoToLoading>::SharedPtr srv_client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AddTwoIntsClientNode>();
  rclcpp::shutdown();
  return 0;
}
