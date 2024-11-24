#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav2_apps/srv/go_to_loading.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

class ApproachService : public rclcpp::Node {
public:
  ApproachService() : Node("approach_shelf_service") {

    RCLCPP_INFO(this->get_logger(), "Service running.");
    // Callback groups
    service_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    scan_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    odom_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = scan_callback_group_;
    rclcpp::SubscriptionOptions options2;
    options2.callback_group = odom_callback_group_;

    // Services, subscribers, and publishers
    service_ = this->create_service<nav2_apps::srv::GoToLoading>(
        "approach_shelf",
        std::bind(&ApproachService::service_callback, this,
                  std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_default, service_callback_group_);
    scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&ApproachService::scan_callback, this, std::placeholders::_1),
        options1);
    odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&ApproachService::odom_callback, this, std::placeholders::_1),
        options2);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);
    elevator_publisher =
        this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);
    leg_distance_ = 0.0;
    intensity_threshold = 2000;
  }

private:
  void service_callback(
      const std::shared_ptr<nav2_apps::srv::GoToLoading::Request> request,
      std::shared_ptr<nav2_apps::srv::GoToLoading::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Service called");

    if (request->attach_to_shelf) {
      RCLCPP_INFO(this->get_logger(), "Starting alignment with cart");
      // Detecting cart and started approach if detected
      if (std::abs(leg_distance_ - 0.7) < 0.1) {
        approach = "align";
        while (!success) {
          if (success) {
            break;
          }
        }
        response->complete = true;
        rclcpp::shutdown();
      } else {
        response->complete = false;
        rclcpp::shutdown();
      }
    }
    rclcpp::shutdown();
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    double angle1, angle2;
    b = c = 0;
    // Scanning intensities from left side
    for (size_t i = 0; i < msg->intensities.size(); ++i) {
      if (msg->intensities[i] > intensity_threshold) {
        angle1 = msg->angle_min + i * msg->angle_increment;
        b = msg->ranges[i];
        break;
      }
    }

    // Scanning intensities from right side
    for (int i = msg->intensities.size() - 1; i >= 0; --i) {
      if (msg->intensities[i] > intensity_threshold) {
        angle2 = msg->angle_min + i * msg->angle_increment;
        c = msg->ranges[i];
        break;
      }
    }
    laser_angle = angle2;
    theta = std::abs(angle1 - angle2);

    // Confirming cart presence
    leg_distance_ = sqrt(c * c + b * b - 2 * c * b * cos(theta));
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    // Coordinates of points B and C
    double xC = 0.0;
    double yC = c;
    double xB = b * sin(theta);
    double yB = b * cos(theta);

    // Midpoint M coordinates
    double xM = (xB + xC) / 2.0;
    double yM = (yB + yC) / 2.0;

    // Rotating coordinates to account for laser beam angle relative to robot
    double yM2 =
        yM * cos(1.57079 - laser_angle) - xM * sin(1.57079 - laser_angle);
    double xM2 =
        yM * sin(1.57079 - laser_angle) + xM * cos(1.57079 - laser_angle);

    double angle_AM = std::atan2(yM2, xM2);

    // Creating transform
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "robot_front_laser_base_link";
    t.child_frame_id = "cart_frame";

    t.transform.translation.x = xM2;
    t.transform.translation.y = yM2;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = std::sin(angle_AM / 2);
    t.transform.rotation.w = std::cos(angle_AM / 2);

    tf_broadcaster_->sendTransform(t);

    // Aligning with cart
    error_yaw = -std::atan2(yM2, xM2);
    error_distance = std::sqrt(std::pow(yM2, 2) + std::pow(xM2, 2));

    auto vel_msg = geometry_msgs::msg::Twist();
    if (approach == "align") {
      control = true;
      if (error_distance > 0.02) {
        xx = 0.2 * error_distance + 0.1;
        zz = 0.3 * error_yaw;
      } else {
        approach = "forward";
      }
    }

    if ((approach == "forward") && (error_distance < 0.45)) {
      xx = 0.2;
      zz = 0.0;
    } else if ((approach == "forward") && (error_distance > 0.45)) {
      xx = zz = 0.0;
      x1 = msg->pose.pose.position.x;
      y1 = msg->pose.pose.position.y;
      approach = "lift";
    }

    if (approach == "lift") {
      auto elevator_msg = std_msgs::msg::String();
      elevator_publisher->publish(elevator_msg);
      start_time = this->get_clock()->now();
      RCLCPP_INFO(this->get_logger(), "Lifting cart");
      approach = "wait";
    }

    if (approach == "wait") {
      end_time = this->get_clock()->now();
      auto time_diff = end_time - start_time;
      double seconds = time_diff.seconds();
      if (seconds > 3.0) {
        approach = "drive_back";
        RCLCPP_INFO(this->get_logger(), "Driving back");
      }
    }

    if (approach == "drive_back") {
      xx = -0.2;
      double dx = msg->pose.pose.position.x - x1;
      double dy = msg->pose.pose.position.y - y1;
      double distance = sqrt(dx * dx + dy * dy);
      if (distance > 0.65) {
        approach = "end";
      }
    }

    if (control) {
      vel_msg.linear.x = xx;
      vel_msg.angular.z = zz;
      cmd_vel_publisher->publish(vel_msg);
      if (approach == "end") {
        control = false;
        RCLCPP_INFO(this->get_logger(), "Finished attachment with cart");
        success = true;
      }
    }
  }

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;

  rclcpp::Service<nav2_apps::srv::GoToLoading>::SharedPtr service_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_publisher;

  double leg_distance_;
  double b, c, theta, laser_angle;
  double intensity_threshold;
  double error_yaw, error_distance, xx, zz;
  double x1, y1;
  rclcpp::Time start_time, end_time;
  std::string approach;
  bool success;
  bool control;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ApproachService>();
  rclcpp::executors::MultiThreadedExecutor executor1;
  executor1.add_node(node);
  executor1.spin();
  rclcpp::shutdown();
  return 0;
}