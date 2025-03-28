#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/detail/int32__struct.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shelf_detector/srv/go_to_loading.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class ApproachService : public rclcpp::Node {
public:
  ApproachService() : Node("transform_node") {

    RCLCPP_INFO(this->get_logger(), "Service node running.");
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

    // Robot parameters
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Sets the sim/real mode";
    this->declare_parameter<int>("mode", 0, param_desc);
    this->get_parameter("mode", mode_param);
    if (mode_param == 0) {
      vel_topic = "diffbot_base_controller/cmd_vel_unstamped";
      laser_link = 0.23;
      dock_dis = 0.65;
      leg_min = 0.7;
      leg_max = 0.8;
    } else {
      vel_topic = "/cmd_vel";
      laser_link = 0.25;
      dock_dis = 0.65;
      leg_min = 0.50;
      leg_max = 0.76;
    }

    // Services, subscribers, and publishers
    service_ = this->create_service<shelf_detector::srv::GoToLoading>(
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
    cmd_vel_publisher =
        this->create_publisher<geometry_msgs::msg::Twist>(vel_topic, 10);
    elevator_publisher =
        this->create_publisher<std_msgs::msg::String>("/elevator_up", 10);
    command_publisher =
        this->create_publisher<std_msgs::msg::Int32>("/command_topic", 10);
    leg_distance_ = 0.0;
    intensity_threshold = 2000;
    step_ = counter_ = 0;
  }

private:
  void service_callback(
      const std::shared_ptr<shelf_detector::srv::GoToLoading::Request> request,
      std::shared_ptr<shelf_detector::srv::GoToLoading::Response> response) {
    // RCLCPP_INFO(this->get_logger(), "Service called");
    if (request->attach_to_shelf == 1) {
      response->complete = shelf_;
      if (shelf_) {
        RCLCPP_INFO(this->get_logger(), "Shelf detected");
      }
    } else if (request->attach_to_shelf == 10) {
      RCLCPP_INFO(this->get_logger(), "1/11 Rotating towards park position");
      step_ = 10;
      response->complete = true;
      RCLCPP_INFO(this->get_logger(), "Finished attachment with cart");
    } else if (request->attach_to_shelf == 0) {
      RCLCPP_INFO(this->get_logger(), "1/11 Rotating towards park position");
      step_ = 130;
      response->complete = true;
      RCLCPP_INFO(this->get_logger(), "Finished attachment with cart");
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    b = c = 0;
    // Scanning intensities from left side
    for (size_t i = 0; i < msg->intensities.size(); i += 2) {
      if (msg->intensities[i] > intensity_threshold && msg->ranges[i] < 2) {
        angle1 = msg->angle_min + i * msg->angle_increment;
        b = msg->ranges[i]; // long
        break;
      }
    }

    // Scanning intensities from right side
    for (int i = msg->intensities.size() - 1; i >= 0; i -= 2) {
      if (msg->intensities[i] > intensity_threshold && msg->ranges[i] < 2) {
        angle2 = msg->angle_min + i * msg->angle_increment;
        c = msg->ranges[i]; // short
        break;
      }
    }

    // Confirming cart presence
    theta = std::abs(angle1 - angle2);
    leg_distance_ = sqrt(c * c + b * b - 2 * c * b * cos(theta));
    if (leg_distance_ < leg_min || leg_distance_ > leg_max) {
      shelf_ = false;
    } else {
      shelf_ = true;
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Main variables
    double xxx, yyy, xM2, yM2;

    if (shelf_) {
      // Coordinates of points B and C
      double xC = 0.0;
      double yC = c;
      double xB = b * sin(theta);
      double yB = b * cos(theta);

      // Midpoint M coordinates
      double xM = (xB + xC) / 2.0;
      double yM = (yB + yC) / 2.0;

      // Rotating coordinates to account for laser beam angle relative to robot
      yM2 = yM * cos(M_PI / 2 - angle2) - xM * sin(M_PI / 2 - angle2);
      xM2 = yM * sin(M_PI / 2 - angle2) + xM * cos(M_PI / 2 - angle2);
      ///////////////////////////////////////
      double al_ = abs(angle1); // left angle
      double ar_ = abs(angle2); // right angle
      double A = sqrt(b * b + c * c - 2 * b * c * cos(theta));
      double X_ = asin((b * sin(theta)) / A);
      double X__ = M_PI - theta - X_;

      if (c < b) {
        if (angle1 < 0 && angle2 < 0) {
          double X1 = std::min(X_, X__);
          double X2 = (M_PI / 2) - X1;
          double X3 = M_PI - al_;
          th_w = X2 + X3;
        } else if (angle1 < 0 && angle2 > 0) {
          double X1 = std::max(X_, X__);
          double X2 = M_PI - (ar_ + X1);
          th_w = (3 * M_PI / 2) - X2;
        } else if (angle1 > 0 && angle2 > 0) {
          double X1 = std::min(X_, X__);
          double X2 = (M_PI / 2) - X1;
          th_w = M_PI + X2 + al_;
        }
      } else if (b < c) {
        if (angle2 > 0 && angle1 > 0) {
          double X1 = std::min(X_, X__);
          double X2 = (M_PI / 2) - X1;
          double X3 = M_PI - ar_;
          th_w = X2 + X3;
        } else if (angle2 > 0 && angle1 < 0) {
          double X1 = std::max(X_, X__);
          double X2 = M_PI - (al_ + X1);
          th_w = (3 * M_PI / 2) - X2;
        } else if (angle2 < 0 && angle1 < 0) {
          double X1 = std::min(X_, X__);
          double X2 = (M_PI / 2) - X1;
          th_w = M_PI + X2 + ar_;
        }
        th_w = -th_w;
      }

      tf2::Quaternion q2;
      q2.setRPY(0.0, 0.0, th_w);

      yyy = -0.65 * sin(-th_w);
      xxx = 0.65 * cos(-th_w);
      xxx = xxx + xM2;
      yyy = yyy + yM2;
      /////////////////////////////////////////
    }

    // Creating transform
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "robot_front_laser_base_link";
    t.child_frame_id = "cart_frame_3";

    t.transform.translation.x = xxx;
    t.transform.translation.y = yyy;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;
    if (shelf_) {
      tf_broadcaster_->sendTransform(t);
    }

    // Control switch
    switch (step_) {

    case 0: {
      /*if (counter_ % 10 == 0) {
        RCLCPP_INFO(this->get_logger(), "\nLeg dis is %.2f\nShelf: %d",
                    leg_distance_, shelf_);
      }
      counter_ += 1;*/
      break;
    }

    // Rotating towards park position
    case 10: {
      control = true;
      error_yaw = -std::atan2(yyy, xxx);
      if (std::abs(error_yaw) > 0.15) {
        xx = 0.0;
        zz = (error_yaw > 0) ? std::max(0.5 * error_yaw, 0.1)
                             : std::min(0.5 * error_yaw, -0.1);
      } else {
        step_ = 20; // Next step
        RCLCPP_INFO(this->get_logger(), "2/11 Moving towards park position");
        xx = zz = 0.0;
      }
      break;
    }

    // Moving towards park position
    case 20: {
      error_yaw = -std::atan2(yyy, xxx);
      error_distance = std::sqrt(std::pow(xxx, 2) + std::pow(yyy, 2));
      if (error_distance > 0.05) {
        xx = std::max(0.2 * error_distance, 0.1);
        zz = (error_yaw > 0) ? std::max(0.5 * error_yaw, 0.1)
                             : std::min(0.5 * error_yaw, -0.1);
      } else {
        step_ = 30; // Next step
        RCLCPP_INFO(this->get_logger(), "3/11 Rotating towards furthest leg");
        xx = zz = 0.0;
      }
      break;
    }

    // Rotating towards furthest leg
    case 30: {
      if (std::max(b, c) == c) {
        error_yaw = -angle2;
      } else {
        error_yaw = -angle1;
      }
      if (std::abs(error_yaw) > 0.15) {
        xx = 0.0;
        zz = (error_yaw > 0) ? std::max(0.5 * error_yaw, 0.1)
                             : std::min(0.5 * error_yaw, -0.1);
      } else {
        step_ = 40; // Next step
        RCLCPP_INFO(this->get_logger(), "4/11 Moving towards furthest leg");
        xx = zz = 0.0;
      }
      break;
    }

    // Moving towards furthest leg
    case 40: {
      if (std::max(b, c) == c) {
        error_yaw = -angle2;
        error_distance = c - b;
      } else {
        error_yaw = -angle1;
        error_distance = b - c;
      }
      if (error_distance > 0.01) {
        xx = 0.1;
        zz = (error_yaw > 0) ? std::max(0.5 * error_yaw, 0.1)
                             : std::min(0.5 * error_yaw, -0.1);
      } else {
        step_ = 50;
        // Next step
        RCLCPP_INFO(this->get_logger(), "5/11 Correcting for lidar position");
        xx = zz = 0.0;
        x1 = msg->pose.pose.position.x;
        y1 = msg->pose.pose.position.y;
      }
      break;
    }

    // Correcting for laser link position
    case 50: {
      error_distance = std::sqrt(std::pow(x1 - msg->pose.pose.position.x, 2) +
                                 std::pow(y1 - msg->pose.pose.position.y, 2));
      if (error_distance < laser_link) {
        xx = 0.1;
        zz = 0.0;
      } else {
        step_ = 60;
        // Next step
        xx = zz = 0.0;
        RCLCPP_INFO(this->get_logger(), "6/11 Rotating towards TF");
      }
      break;
    }

    // Rotating towards TF
    case 60: {
      error_yaw = (shelf_) ? -std::atan2(yM2, xM2) : -angle1;
      if (std::abs(error_yaw) > 0.02) {
        zz = (error_yaw > 0) ? std::max(0.5 * error_yaw, 0.1)
                             : std::min(0.5 * error_yaw, -0.1);
      } else {
        step_ = 70;
        // Next step
        xx = zz = 0.0;
        RCLCPP_INFO(this->get_logger(), "7/11 Moving towards TF");
      }
      break;
    }

    // Moving towards TF
    case 70: {
      error_yaw = -std::atan2(yM2, xM2);
      error_distance = std::sqrt(std::pow(xM2, 2) + std::pow(yM2, 2));

      xx = std::max(0.2 * error_distance, 0.1);
      zz = (error_yaw > 0) ? std::max(0.5 * error_yaw, 0.1)
                           : std::min(0.5 * error_yaw, -0.1);
      if (error_distance < 0.15) {
        step_ = 80;
        // Next step
        xx = zz = 0.0;
        // RCLCPP_INFO(this->get_logger(), "Starting final alignment");
        RCLCPP_INFO(this->get_logger(), "8/11 Final alignment");
      }
      break;
    }

      // Final alignment
    case 80: {
      error_yaw = -std::atan2(yM2, xM2);

      zz = 0.5 * error_yaw;
      if (std::abs(error_yaw) < 0.1) {
        step_ = 90;
        // Next step
        xx = zz = 0.0;
        RCLCPP_INFO(this->get_logger(), "9/11 Docking using odometry");
        x1 = msg->pose.pose.position.x;
        y1 = msg->pose.pose.position.y;
      }
      break;
    }
      // Odom docking before lift
    case 90: {
      error_distance = std::sqrt(std::pow(x1 - msg->pose.pose.position.x, 2) +
                                 std::pow(y1 - msg->pose.pose.position.y, 2));
      if (error_distance < dock_dis) {
        xx = 0.1;
        zz = 0.0;
      } else {
        step_ = 100; // Next step
        xx = zz = 0.0;
        auto elevator_msg = std_msgs::msg::String();
        elevator_publisher->publish(elevator_msg);
        start_time = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), "10/11 Lifting cart");
      }
      break;
    }

    case 100: {
      end_time = this->get_clock()->now();
      if ((end_time - start_time).seconds() > 5) {
        x1 = msg->pose.pose.position.x;
        y1 = msg->pose.pose.position.y;
        step_ = 110;
        RCLCPP_INFO(this->get_logger(), "11/11 Backing up");
      }

      break;
    }

    case 110: {
      error_distance = std::sqrt(std::pow(x1 - msg->pose.pose.position.x, 2) +
                                 std::pow(y1 - msg->pose.pose.position.y, 2));
      if (error_distance < 0.5) {
        xx = -0.1;
        zz = 0.0;
      } else {
        step_ = 120; // Next step
        xx = zz = 0.0;
      }
      break;
    }

    default:
      break;
    }

    if (control) {
      auto vel_msg = geometry_msgs::msg::Twist();
      vel_msg.linear.x = xx;
      vel_msg.angular.z = zz;
      cmd_vel_publisher->publish(vel_msg);
      if (step_ == 120) {
        auto msg = std_msgs::msg::Int32();
        msg.data = 3;
        command_publisher->publish(msg);
        control = false;
        success = true;
      }
    }
  }

  // Callback groups
  rclcpp::CallbackGroup::SharedPtr service_callback_group_;
  rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;

  rclcpp::Service<shelf_detector::srv::GoToLoading>::SharedPtr service_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr elevator_publisher;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr command_publisher;

  // Main variables
  double leg_distance_;
  double b, c, theta;
  double angle1, angle2;
  double intensity_threshold;
  double error_yaw, error_distance, xx, zz;
  double x1, y1;
  int step_, counter_;
  rclcpp::Time start_time, end_time;
  bool shelf_;
  bool success;
  bool control;
  ///////////
  double th_w;
  //////////
  // Robot parameters
  int mode_param;
  std::string vel_topic;
  double laser_link;
  double dock_dis;
  double leg_min, leg_max;
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
