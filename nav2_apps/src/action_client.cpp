#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/int32.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class NavigateToPoseClient : public rclcpp::Node {
public:
  using GoalHandleNavigateToPose =
      rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToPoseClient() : Node("navigate_to_pose_client") {
    // Initialize the action client
    client_ =
        rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

    main_command = this->create_subscription<std_msgs::msg::Int32>(
        "command_topic", 10,
        std::bind(&NavigateToPoseClient::command_callback, this,
                  std::placeholders::_1));
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(200),
                                std::bind(&TimerNode::timerCallback, this));

    // Wait for the action server
  }

private:
  void timerCallback() {
    RCLCPP_INFO(this->get_logger(), "Timer callback triggered!");
    if (command_ == 1) {
      send_goal(0.68, 0.0, 0.0, 1.0);
    }
  }

  // ros2 topic pub -1 /command_topic std_msgs/Int32 "{data: 1}"
  void command_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I received: '%d'", msg->data);

    if (msg->data == 1) {
      command_ = 1;
    }
  }

  void send_goal(double x, double y, double oz, double ow) {

    // Create the goal message
    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->get_clock()->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation.z = oz;
    goal_msg.pose.pose.orientation.w = ow;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    // Configuring goal options
    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NavigateToPoseClient::goal_response_callback, this,
                  std::placeholders::_1);
    /*send_goal_options.feedback_callback =
        std::bind(&NavigateToPoseClient::feedback_callback, this,
                  std::placeholders::_1, std::placeholders::_2);*/
    send_goal_options.result_callback = std::bind(
        &NavigateToPoseClient::result_callback, this, std::placeholders::_1);

    // Send the goal
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

  /*void feedback_callback(
      GoalHandleNavigateToPose::SharedPtr,
      const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
     RCLCPP_INFO(this->get_logger(), "Feedback: Current position (%.2f,
     %.2f)",
                 feedback->current_pose.pose.position.x,
                 feedback->current_pose.pose.position.y);
  }*/

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

    // rclcpp::shutdown();
  }

  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr main_command;
  rclcpp::TimerBase::SharedPtr timer_;

  int command_;
  bool waiting_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigateToPoseClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
