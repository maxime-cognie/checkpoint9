#ifndef ATTACH_CLIENT_HPP
#define ATTACH_CLIENT_HPP

#include "my_components/visibility_control.h"

#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

using GoToLoading = attach_shelf::srv::GoToLoading;

namespace my_components
{
class AttachClient : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  AttachClient(const rclcpp::NodeOptions options);

private:
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<GoToLoading>::SharedPtr client_;
};

} //namespace composition

#endif // ATTACH_CLIENT_HPP