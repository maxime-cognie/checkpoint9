#include "my_components/preapproach_component.hpp"

#include "angles/angles.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmw/types.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/impl/utils.h"

#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>

template <class T> inline int sgn(T v) { return (v > T(0)) - (v < T(0)); }

using namespace std::chrono_literals;

namespace my_components
{

PreApproach::PreApproach(const rclcpp::NodeOptions options) 
  : Node("preapproach_component", options),
  front_dist_(float INFINITY)
{
  using namespace std::placeholders;

  scan_sub_cbg_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options_1;
  options_1.callback_group = scan_sub_cbg_;

  rclcpp::QoS qos_profile_scan_sub(10);
  qos_profile_scan_sub.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);

  auto scan_callback = [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) -> void {
    size_t front_ray_index = msg->ranges.size() / 2;
    this->front_dist_ = msg->ranges[front_ray_index];

  };

  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", qos_profile_scan_sub, scan_callback, options_1);

  odom_sub_cbg_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options_2;
  options_2.callback_group = odom_sub_cbg_;

  rclcpp::QoS qos_profile_odom_sub(10);
  qos_profile_odom_sub.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

  auto odom_callback = [this](const nav_msgs::msg::Odometry::SharedPtr msg) -> void
  {
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    yaw_ = tf2::impl::getYaw(q);
  };

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", qos_profile_odom_sub, odom_callback, options_2);

  cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/diffbot_base_controller/cmd_vel_unstamped", 1);

  timer_cbg_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  timer_ = this->create_wall_timer(
      100ms, std::bind(&PreApproach::timer_callback, this), timer_cbg_);
}

void PreApproach::timer_callback() {
  timer_->cancel();
  obs_param_ = 0.3;
  deg_param_ = -90;
  RCLCPP_INFO(this->get_logger(),
      "The robot is going to move forward until it detects an obstacle in "
      "front of it at %.2f m and it will then rotate for %d degrees",
      obs_param_, deg_param_);

  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = 0.4;

  while (front_dist_ > obs_param_) {
    cmd_vel_pub_->publish(cmd_vel);
    std::this_thread::sleep_for(50ms);
  }

  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = sgn<int>(deg_param_) * 0.4;

  auto target_rad =
      angles::normalize_angle(yaw_ + angles::from_degrees(deg_param_));

  while (std::abs(target_rad - yaw_) > 0.0175) {
    cmd_vel_pub_->publish(cmd_vel);
    std::this_thread::sleep_for(50ms);
  }

  cmd_vel.linear.x = 0.0;
  cmd_vel.angular.z = 0.0;
  cmd_vel_pub_->publish(cmd_vel);
  RCLCPP_INFO(this->get_logger(), "Pre-approach mission done, the robot has stopped");
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)