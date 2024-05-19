#ifndef ATTACH_SERVER_HPP
#define ATTACH_SERVER_HPP

#include "my_components/visibility_control.h"

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

using GoToLoading = attach_shelf::srv::GoToLoading;

namespace my_components
{
class AttachServer : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  AttachServer(const rclcpp::NodeOptions options);

private:
  void srv_callback(const GoToLoading::Request::SharedPtr request,
                    GoToLoading::Response::SharedPtr response);

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  std::pair<float, float> get_pos(
    const sensor_msgs::msg::LaserScan::SharedPtr msg, const int ind);

  std::pair<float, float> get_median(
    const std::pair<float, float> &first_pos,
    const std::pair<float, float> &second_pos);

  rclcpp::CallbackGroup::SharedPtr srv_cbg_;
  rclcpp::CallbackGroup::SharedPtr scan_sub_cbg_;
  rclcpp::Service<GoToLoading>::SharedPtr approach_srv_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  geometry_msgs::msg::TransformStamped transform_stamped_;
  std::shared_ptr<std::pair<float, float>> cart_pos_;

  bool cart_detected_;
  bool publish_tf_;
};

} //namespace composition

#endif // ATTACH_SERVER_HPP