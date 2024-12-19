#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rclcpp/parameter.hpp"
#include <cmath>
#include <future>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shelf_detector/srv/go_to_loading.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using NavigateToPoseMsg = nav2_msgs::action::NavigateToPose;
using GoToLoading = shelf_detector::srv::GoToLoading;
using namespace std::chrono_literals;

class NavigateToPoseClient : public rclcpp::Node {
public:
  using GoalHandleNavigateToPose =
      rclcpp_action::ClientGoalHandle<NavigateToPoseMsg>;

  NavigateToPoseClient() : Node("navigate_to_pose_client") {
    // Initialize the action client
    act_client_ = rclcpp_action::create_client<NavigateToPoseMsg>(
        this, "/navigate_to_pose");
    srv_client_ = this->create_client<GoToLoading>("approach_shelf");

    command_sub = this->create_subscription<std_msgs::msg::Int32>(
        "/command_topic", 10,
        std::bind(&NavigateToPoseClient::command_callback, this,
                  std::placeholders::_1));

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&NavigateToPoseClient::odom_callback, this,
                  std::placeholders::_1));

    cmd_vel_publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    elevator_publisher =
        this->create_publisher<std_msgs::msg::String>("/elevator_down", 10);
    pose_publisher_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);
    param_client_1 = this->create_client<rcl_interfaces::srv::SetParameters>(
        "/global_costmap/global_costmap/set_parameters");
    controller_ = this->create_wall_timer(
        100ms, std::bind(&NavigateToPoseClient::control_callback, this));

    round_ = 1;
    cmd = Command::None;

    // parameters
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Sets the sim/real mode";
    this->declare_parameter<int>("mode", 0, param_desc);
    this->get_parameter("mode", mode_param);

    if (mode_param == 0) {
      M = {2.27, -2.04, 0.92, -2.09, 0.68, 0.04,
           2.49, 0.10,  4.39, 0.11,  5.46, -0.03};
      H1 = {2.27, -2.04, 0.92, -2.09, 0.04, 0.06};
      H2 = {5.46, -0.03, 4.39, 0.11, 2.49, 0.10, 0.68, 0.04, 0.04, 0.06};
      D1 = {0.92, -2.09, 0.68, 0.038, 2.54, 0.26};
      D2 = {4.39, 0.11, 2.54, 0.26};
      vel_topic = "/diffbot_base_controller/cmd_vel_unstamped";
    } else {
      M = {-0.02, -2.01, -0.01, -0.04, 1.09, 0.01, 2.45, -0.01, 3.56, -0.21};
      H1 = {-0.02, -2.01, -0.01, -0.04, -0.58, 0.06};
      H2 = {3.56, -0.21, 2.45, 0.00, 1.09, 0.01, -0.01, -0.04, -0.58, 0.06};
      D1 = {-0.01, -0.04, 1.11, 0.17};
      D2 = {2.45, -0.01, 1.11, 0.17};
      vel_topic = "/cmd_vel";
    }
    cmd_vel_publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Waiting for the /navigate_to_pose action server
    while (!act_client_->wait_for_action_server(1s)) {
      RCLCPP_INFO(this->get_logger(), "Action not available, waiting...");
    }

    while (!srv_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
    }
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;
    current_z = msg->pose.pose.orientation.z;
    current_w = msg->pose.pose.orientation.w;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw_ = yaw;

    if (!initial_) {
      pub_initial();
    }
  }

  void pub_initial() {
    // Create a PoseWithCovarianceStamped message
    auto message = geometry_msgs::msg::PoseWithCovarianceStamped();

    // Set the header and pose
    message.header.frame_id = "map";
    message.header.stamp = this->get_clock()->now();

    message.pose.pose.position.x = -0.58;
    message.pose.pose.position.y = 0.06;
    message.pose.pose.position.z = 0.0;

    message.pose.pose.orientation.x = 0.0;
    message.pose.pose.orientation.y = 0.0;
    message.pose.pose.orientation.z = 0;
    message.pose.pose.orientation.w = 1;

    // Publishing the message
    pose_publisher_->publish(message);
    initial_ = true;
  }

  // ros2 topic pub -1 /command_topic std_msgs/Int32 "{data: 1}"
  void command_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (msg->data == 1) {
      RCLCPP_INFO(this->get_logger(), "I received: '%d'", msg->data);
      cmd = Command::SearchGoal;
    } else if (msg->data == 2) {
      RCLCPP_INFO(this->get_logger(), "I received: '%d'", msg->data);
      cmd = Command::HomeGoal;
    } else if (msg->data == 0) {
      RCLCPP_INFO(this->get_logger(), "I received: '%d'", msg->data);
      cmd = Command::None;
    }
  }

  void control_callback() {
    switch (cmd) {
    // Search mode
    case Command::SearchGoal: {
      // Setting optotions
      search_ = true;
      home_ = false;
      deliver_ = false;
      control_ = true;
      srv_wait_ = false;
      round_ = 1;

      // Finding closest end
      // waypoints = {2.27, -2.04, 0.92, -2.09, 0.68, 0.04,
      //           2.49, 0.10,  4.39, 0.11,  5.46, -0.03};
      waypoints = {-0.02, -2.01, -0.01, -0.04, 1.09,
                   0.01,  2.45,  -0.01, 3.56,  -0.21};
      RCLCPP_INFO(this->get_logger(), "Finding closest end");
      next_waypoint(end_index, goal_index);

      cmd = Command::Rotate;
      RCLCPP_INFO(this->get_logger(), "Starting rotation");

      break;
    }

    // Return to home mode
    case Command::HomeGoal: {
      // Setting options
      home_ = true;
      search_ = false;
      deliver_ = false;
      control_ = true;
      srv_wait_ = true;
      round_ = 2;

      // Finding closest end
      RCLCPP_INFO(this->get_logger(), "Finding closest direction to home");
      // waypoints = {2.27, -2.04, 0.92, -2.09, 0.04, 0.06};
      waypoints = {-0.02, -2.01, -0.01, -0.04, -0.58, 0.06};
      next_waypoint(end_index, goal_index);
      std::pair<double, double> point_1 = {waypoints[goal_index],
                                           waypoints[goal_index + 1]};
      int goal_1 = goal_index;

      // waypoints = {5.46, -0.03, 4.39, 0.11, 2.49, 0.10, 0.68, 0.04, 0.04,
      // 0.06};
      waypoints = {3.56, -0.21, 2.45,  0.00,  1.09,
                   0.01, -0.01, -0.04, -0.58, 0.06};
      next_waypoint(end_index, goal_index);
      std::pair<double, double> point_2 = {waypoints[goal_index],
                                           waypoints[goal_index + 1]};

      double dist_to_point_1 =
          std::hypot(point_1.first - current_x, point_1.second - current_y);

      double dist_to_point_2 =
          std::hypot(point_2.first - current_x, point_2.second - current_y);

      if (dist_to_point_1 < dist_to_point_2) {
        // waypoints = {2.27, -2.04, 0.92, -2.09, 0.04, 0.06};
        waypoints = {-0.02, -2.01, -0.01, -0.04, -0.58, 0.06};
        end_index = waypoints.size() - 2;
        goal_index = goal_1;
      }

      cmd = Command::Rotate;
      RCLCPP_INFO(this->get_logger(), "Starting rotation");

      break;
    }

    case Command::DeliverGoal: {
      // Setting options
      deliver_ = true;
      search_ = false;
      control_ = true;
      srv_wait_ = true;
      round_ = 2;

      // Finding closest end
      RCLCPP_INFO(this->get_logger(), "Finding direction to delivery target");
      // waypoints = {0.92, -2.09, 0.68, 0.038, 2.54, 0.26};
      waypoints = {-0.01, -0.04, 1.11, 0.17};
      next_waypoint(end_index, goal_index);
      std::pair<double, double> point_1 = {waypoints[goal_index],
                                           waypoints[goal_index + 1]};
      int goal_1 = goal_index;

      // waypoints = {4.39, 0.11, 2.54, 0.26};
      waypoints = {2.45, -0.01, 1.11, 0.17};
      next_waypoint(end_index, goal_index);
      std::pair<double, double> point_2 = {waypoints[goal_index],
                                           waypoints[goal_index + 1]};

      double dist_to_point_1 =
          std::hypot(point_1.first - current_x, point_1.second - current_y);

      double dist_to_point_2 =
          std::hypot(point_2.first - current_x, point_2.second - current_y);

      if (dist_to_point_1 < dist_to_point_2) {
        // waypoints = {0.92, -2.09, 0.68, 0.038, 2.54, 0.26};
        waypoints = {-0.01, -0.04, 1.11, 0.17};
        end_index = waypoints.size() - 2;
        goal_index = goal_1;
      }

      cmd = Command::Rotate;
      RCLCPP_INFO(this->get_logger(), "Starting rotation");

      break;
    }

    case Command::Rotate: {
      // Checking for ongoing goal
      if (nav_wait_ == true && !success_ && !aborted_) {
        break;
      }

      // Rotating towards goal
      double angle_to_waypoint =
          std::atan2(waypoints[goal_index + 1] - current_y,
                     waypoints[goal_index] - current_x);
      // yaw_ = (yaw_ > 0) ? yaw_ - M_PI : yaw_ + M_PI;
      yaw_error = angle_to_waypoint - yaw_;
      if (std::abs(yaw_error) > 0.1) {
        zz = (yaw_error > 0) ? std::max(0.5 * yaw_error, 0.5)
                             : std::min(0.5 * yaw_error, -0.5);
      } else {
        zz = 0.0;
        control_ = false;
        nav_wait_ = false;
        cmd = Command::SendGoals;
        RCLCPP_INFO(this->get_logger(), "Orientation reached, sending goal");
      }
      break;
    }

    case Command::SendGoals: {
      // Sending current goal
      if (!nav_wait_) {
        RCLCPP_INFO(this->get_logger(), "\nSending:\nEnd: %d\nGoal: %d",
                    end_index / 2 + 1, goal_index / 2 + 1);
        send_goal(waypoints[goal_index], waypoints[goal_index + 1]);
        RCLCPP_INFO(this->get_logger(), "Goal sent");
        nav_wait_ = true;
      }

      // Checking success and switching to next goal
      if (success_ || aborted_) {
        if (end_index == 0) {
          RCLCPP_INFO(this->get_logger(), "Reached, moving to next");
          goal_index = (goal_index == end_index) // || distance_error > 0.4)
                           ? -10
                           : goal_index - 2;
        } else if (static_cast<unsigned int>(end_index) ==
                   waypoints.size() - 2) {
          RCLCPP_INFO(this->get_logger(), "Reached, moving to next");
          goal_index = (static_cast<unsigned int>(goal_index) ==
                        (waypoints.size() - 2)) // || distance_error > 0.4)
                           ? -20
                           : goal_index + 2;
        }
        nav_wait_ = false;
        success_ = false;
        aborted_ = false;
        control_ = true;
        cmd = Command::Rotate;
      }

      // Starting second round or ending search
      if (goal_index == -10) {
        end_index = waypoints.size() - 2;
        goal_index = 2;
        cmd = Command::Rotate;
        round_ += 1;
      } else if (goal_index == -20) {
        end_index = 0;
        goal_index = waypoints.size() - 4;
        cmd = Command::Rotate;
        round_ += 1;
      }

      if (round_ > 2) {
        if (search_) {
          RCLCPP_INFO(this->get_logger(), "Shelf wasn't found");
          cmd = Command::None;
        } else if (home_) {
          cmd = Command::None;
        } else if (deliver_) {
          x1 = current_x;
          y1 = current_y;
          cmd = Command::Forward;
        }
      }

      // Trying to detect shelf
      if (!srv_wait_) {
        auto request = std::make_shared<GoToLoading::Request>();
        request->attach_to_shelf = 1;

        auto result_future = srv_client_->async_send_request(
            request, std::bind(&NavigateToPoseClient::srv_clbk, this,
                               std::placeholders::_1));
        srv_wait_ = true;
      }

      break;
    }

    case Command::Attach: {
      // Attach, resize footprint and go to deliver <-----------///////
      if (!srv_wait_) {
        // Starting attachement
        auto request = std::make_shared<GoToLoading::Request>();
        request->attach_to_shelf = 10;

        auto result_future = srv_client_->async_send_request(
            request, std::bind(&NavigateToPoseClient::attach_clbk, this,
                               std::placeholders::_1));
        srv_wait_ = true;
      }
      break;
    }
      //
    case Command::Forward: {
      error_distance =
          std::sqrt(std::pow(x1 - current_x, 2) + std::pow(y1 - current_y, 2));
      if (error_distance < 0.8) {
        xx = 0.1;
      } else {
        xx = 0.0;
        start_time = this->get_clock()->now();
        cmd = Command::Lower;
      }
      break;
    }

    case Command::Lower: {
      end_time = this->get_clock()->now();
      auto elevator_msg = std_msgs::msg::String();
      elevator_publisher->publish(elevator_msg);
      if ((end_time - start_time).seconds() > 5) {
        x1 = current_x;
        y1 = current_y;
        control_ = true;
        resize_function(0.25);
        cmd = Command::Backup;
      }
      break;
    }

    case Command::Backup: {
      error_distance =
          std::sqrt(std::pow(x1 - current_x, 2) + std::pow(y1 - current_y, 2));
      if (error_distance < 1.0) {
        xx = -0.1;
        zz = 0.0;
      } else {
        cmd = Command::None;
        xx = zz = 0.0;
      }
      break;
    }

    default:
      break;
    }

    if (control_) {
      geometry_msgs::msg::Twist cmd_msg;
      cmd_msg.linear.x = xx;
      cmd_msg.angular.z = zz;
      cmd_vel_publisher->publish(cmd_msg);
      if (cmd == Command::None) {
        control_ = false;
      }
    }
  }

  void next_waypoint(int &end_index, int &goal_index) {
    // Determining the closest end waypoint
    double target_x, target_y;
    if (search_) {
      RCLCPP_INFO(this->get_logger(), "Calculating waypoint");
      double dist_to_first =
          std::hypot(waypoints[0] - current_x, waypoints[1] - current_y);
      double dist_to_last =
          std::hypot(waypoints[waypoints.size() - 2] - current_x,
                     waypoints[waypoints.size() - 1] - current_y);

      target_x = (dist_to_first < dist_to_last)
                     ? waypoints[0]
                     : waypoints[waypoints.size() - 2];
      target_y = (dist_to_first < dist_to_last)
                     ? waypoints[1]
                     : waypoints[waypoints.size() - 1];

      // End x index
      end_index = (dist_to_first < dist_to_last) ? 0 : waypoints.size() - 2;
    } else {
      end_index = waypoints.size() - 2;
      target_x = waypoints[waypoints.size() - 2];
      target_y = waypoints[waypoints.size() - 1];
    }

    // Finding the closest waypoint in the direction of the closest end
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

          // Goal x index
          goal_index = i;
        }
      }
    }
  }

  void send_goal(double x, double y) {

    // Create the goal message
    auto goal_msg = NavigateToPoseMsg::Goal();
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.header.stamp = this->get_clock()->now();
    goal_msg.pose.pose.position.x = x;
    goal_msg.pose.pose.position.y = y;
    goal_msg.pose.pose.orientation.z = current_z;
    goal_msg.pose.pose.orientation.w = current_w;
    if (x == 1.11 && y == 0.17) {
      goal_msg.pose.pose.orientation.z = 0.70;
      goal_msg.pose.pose.orientation.w = 0.70;
    }

    if (x == 2.54 && y == 0.26) {
      goal_msg.pose.pose.orientation.z = 0.707;
      goal_msg.pose.pose.orientation.w = 0.707;
    }

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    // Configuring response and result callback
    auto send_goal_options =
        rclcpp_action::Client<NavigateToPoseMsg>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&NavigateToPoseClient::goal_response_callback, this,
                  std::placeholders::_1);
    send_goal_options.result_callback = std::bind(
        &NavigateToPoseClient::act_result_clbk, this, std::placeholders::_1);

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

  void act_result_clbk(const GoalHandleNavigateToPose::WrappedResult &result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
      success_ = true;
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "Goal was canceled");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(this->get_logger(), "Goal was aborted");
      aborted_ = true;
      break;
    default:
      RCLCPP_INFO(this->get_logger(), "Unknown result code");
      break;
    }
  }

  void resize_function(double size) {
    // Resizing
    auto request_1 =
        std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    request_1->parameters.push_back(
        rclcpp::Parameter("robot_radius", size).to_parameter_msg());

    auto result_future_1 = param_client_1->async_send_request(
        request_1, std::bind(&NavigateToPoseClient::resize_response_1, this,
                             std::placeholders::_1));
  }

  void srv_clbk(rclcpp::Client<GoToLoading>::SharedFuture future) {
    auto status = future.wait_for(1s);

    if (status == std::future_status::ready) {
      auto response = future.get();
      if (response->complete) {
        RCLCPP_INFO(this->get_logger(), "Search result: %d",
                    response->complete);
        send_goal(1000, 1000);
        cmd = Command::Attach;
      }
      srv_wait_ = false;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

  void attach_clbk(rclcpp::Client<GoToLoading>::SharedFuture future) {
    auto status = future.wait_for(1s);

    if (status == std::future_status::ready) {
      auto response = future.get();
      if (response->complete) {
        RCLCPP_INFO(this->get_logger(), "Attachment result: %d",
                    response->complete);
        resize_function(0.26);
        cmd = Command::DeliverGoal;
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

  void resize_response_1(
      rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedFuture future) {
    auto status = future.wait_for(std::chrono::seconds(1));

    if (status == std::future_status::ready) {
      auto response = future.get();
      for (const auto &result : response->results) {
        if (result.successful) {
          RCLCPP_INFO(this->get_logger(),
                      "global map parameter set successfully.");
        } else {
          RCLCPP_WARN(this->get_logger(), "Failed to set parameter: %s",
                      result.reason.c_str());
        }
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }

  rclcpp_action::Client<NavigateToPoseMsg>::SharedPtr act_client_;
  rclcpp::Client<GoToLoading>::SharedPtr srv_client_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr command_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_publisher;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
      pose_publisher_;
  rclcpp::TimerBase::SharedPtr controller_;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr param_client_1;

  enum class Command {
    None,
    SearchGoal,
    HomeGoal,
    DeliverGoal,
    Rotate,
    SendGoals,
    Attach,
    Forward,
    Lower,
    Backup
  };
  Command cmd;

  std::vector<double> waypoints, M, H1, H2, D1, D2;
  rclcpp::Time start_time, end_time;
  std::string vel_topic;
  double current_x, current_y, current_z, current_w, yaw_;
  double yaw_error, error_distance, xx, zz, x1, y1;
  int command_, end_index, goal_index, round_, mode_param;
  bool search_, home_, deliver_, control_, aborted_, success_, nav_wait_,
      srv_wait_, shelf_, initial_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigateToPoseClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
