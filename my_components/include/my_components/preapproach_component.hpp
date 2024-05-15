#ifndef PREAPPROACH_COMPONENT_HPP
#define PREAPPROACH_COMPONENT_HPP

#include "my_components/visibility_control.h"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/impl/utils.h"

namespace my_components
{
class PreApproach : public rclcpp::Node
{
public:
  PreApproach(const rclcpp::NodeOptions options);

private:
  void timer_callback();

  rclcpp::CallbackGroup::SharedPtr scan_sub_cbg_;
  rclcpp::CallbackGroup::SharedPtr odom_sub_cbg_;
  rclcpp::CallbackGroup::SharedPtr timer_cbg_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  float front_dist_;
  float obs_param_;
  int16_t deg_param_;
  tf2Scalar yaw_;
};

} // namespace composition

#endif // PREAPPROACH_COMPONENT_HPP