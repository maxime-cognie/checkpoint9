#include "my_components/attach_client.hpp"

#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"


using GoToLoading = attach_shelf::srv::GoToLoading;
using namespace std::chrono_literals;

namespace my_components {
using namespace std::placeholders;
AttachClient::AttachClient(const rclcpp::NodeOptions options)
    : Node("attach_client_component", options)
{
  client_ = this->create_client<GoToLoading>("approach_shelf");

  timer_ = this->create_wall_timer(100ms,
    std::bind(&AttachClient::timer_callback, this));
}

void AttachClient::timer_callback() {
  timer_->cancel();
  while (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Client interrupted while waiting for service. Terminating...");
      return;
    }
    RCLCPP_INFO(this->get_logger(),
                "Service Unavailable. Waiting for Service...");
  }

  auto request = std::make_shared<GoToLoading::Request>();

  request->attach_to_shelf = true;

  auto result_future = client_->async_send_request(request);

  if (result_future.wait_for(10s) == std::future_status::ready) {
    auto result = result_future.get();
    if(result->complete) {
      RCLCPP_INFO(this->get_logger(), "shelf attachment done with success");
    }
    else {
      RCLCPP_INFO(this->get_logger(), "shelf attachment failed");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(),
                  "Failed to call service /approach_shelf");
  }
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient);