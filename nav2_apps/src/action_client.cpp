#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class NavigateToPoseClient : public rclcpp::Node {
public:
  using GoalHandleNavigateToPose =
      rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToPoseClient() : Node("navigate_to_pose_client") {
    // Initialize the action client
    client_ =
        rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

    // Wait for the action server
    if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      return;
    }

    // Send the goal
    send_goal();
  }

private:
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  void send_goal() {
    // Create the goal message
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->get_clock()->now();
    goal_msg.pose.pose.position.x = 2.45;
    goal_msg.pose.pose.position.y = 0.00;
    goal_msg.pose.pose.orientation.z = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal to move to (2.0, 3.0)");

    // Configure goal options
    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NavigateToPoseClient::goal_response_callback, this,
                  std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(&NavigateToPoseClient::feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback = std::bind(
        &NavigateToPoseClient::result_callback, this, std::placeholders::_1);

    // Send the goal asynchronously
    client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(
      const GoalHandleNavigateToPose::SharedPtr goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by the server, waiting for result");
    }
  }

  void feedback_callback(
      GoalHandleNavigateToPose::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Feedback: Current position (%.2f, %.2f)",
                feedback->current_pose.pose.position.x,
                feedback->current_pose.pose.position.y);
  }

  void result_callback(const GoalHandleNavigateToPose::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "Goal was canceled");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(this->get_logger(), "Goal was aborted");
      break;
    default:
      RCLCPP_INFO(this->get_logger(), "Unknown result code");
      break;
    }

    rclcpp::shutdown();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigateToPoseClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
