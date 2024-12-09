#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using NavigateToPose = nav2_msgs::action::NavigateToPose;

class NavigateToPoseClient : public rclcpp::Node {
public:
  using GoalHandleNavigateToPose =
      rclcpp_action::ClientGoalHandle<NavigateToPose>;

  NavigateToPoseClient() : Node("navigate_to_pose_client") {
    // Initialize the action client
    act_client_ =
        rclcpp_action::create_client<NavigateToPose>(this, "/navigate_to_pose");

    command_sub = this->create_subscription<std_msgs::msg::Int32>(
        "/command_topic", 10,
        std::bind(&NavigateToPoseClient::command_callback, this,
                  std::placeholders::_1));

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&NavigateToPoseClient::odom_callback, this,
                  std::placeholders::_1));
    cmd_vel_publisher =
        this->create_publisher<geometry_msgs::msg::Twist>(vel_topic, 10);
    controller_ =
        this->create_wall_timer(std::chrono::milliseconds(200),
                                std::bind(&TimerNode::control_callback, this));

    waypoints = {0.68, 0.04, 0.92, -2.09, 2.27, -2.04,
                 2.49, 0.10, 4.39, 0.11,  5.46, -0.03};

    // Waiting for the /navigate_to_pose action server
    if (!act_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      return;
    }
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw_ = yaw;
  }

  // ros2 topic pub -1 /command_topic std_msgs/Int32 "{data: 1}"
  void command_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (msg->data == 1) {
      RCLCPP_INFO(this->get_logger(), "I received: '%d'", msg->data);
      command_ = 1;
    }
  }

  void control_callback() {
    if (command_ == 1) {
      int end_index, goal_index;
      next_waypoint(end_index, goal_index);

      // Rotating towards the identified waypoint
      double angle_to_waypoint =
          std::atan2(waypoints[goal_index + 1] - current_y,
                     waypoints[goal_index] - current_x);
      double yaw_error = angle_to_waypoint - yaw_;

      // Publishing twist message
      geometry_msgs::msg::Twist cmd_msg;
      cmd_msg.linear.x = 0.0;        // No forward movement, just rotation
      cmd_msg.angular.z = yaw_error; // Rotate towards waypoint
      cmd_vel_publisher->publish(cmd_msg);

      command_ = (std::abs(yaw_error) > 0.2) ? 1 : 0;
    }
  }

  void next_waypoint(int &end_index, int &goal_index) {

    // Determining the closest end waypoint
    double dist_to_first =
        std::hypot(waypoints[0] - current_x, waypoints[1] - current_y);
    double dist_to_last =
        std::hypot(waypoints[waypoints.size() - 2] - current_x,
                   waypoints[waypoints.size() - 1] - current_y);

    double target_x = (dist_to_first < dist_to_last)
                          ? waypoints[0]
                          : waypoints[waypoints.size() - 2];
    double target_y = (dist_to_first < dist_to_last)
                          ? waypoints[1]
                          : waypoints[waypoints.size() - 1];

    // End x index
    end_index = (dist_to_first < dist_to_last) ? 0 : waypoints.size() - 2;

    // Finding the closest waypoint in the direction of the closest end
    double closest_x = target_x;
    double closest_y = target_y;
    double min_distance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < waypoints.size(); i += 2) {
      double wp_x = waypoints[i];
      double wp_y = waypoints[i + 1];

      double vector_to_wp_x = wp_x - current_x;
      double vector_to_wp_y = wp_y - current_y;
      double vector_to_target_x = target_x - current_x;
      double vector_to_target_y = target_y - current_y;

      double dot_product = vector_to_wp_x * vector_to_target_x +
                           vector_to_wp_y * vector_to_target_y;
      double magnitude_wp = std::hypot(vector_to_wp_x, vector_to_wp_y);
      double magnitude_target =
          std::hypot(vector_to_target_x, vector_to_target_y);
      double angle = std::acos(dot_product / (magnitude_wp * magnitude_target));

      if (angle <= M_PI_2) {
        double distance = std::hypot(vector_to_wp_x, vector_to_wp_y);
        if (distance < min_distance) {
          min_distance = distance;
          closest_x = wp_x;
          closest_y = wp_y;

          // Goal x index
          goal_index = i;
        }
      }
    }

    return 0;
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
    act_client_->async_send_goal(goal_msg, send_goal_options);
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

  rclcpp_action::Client<NavigateToPose>::SharedPtr act_client_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr command_sub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
  rclcpp::TimerBase::SharedPtr controller_;

  std::vector<double> waypoints;
  double current_x, current_y, yaw_;
  int command_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigateToPoseClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
