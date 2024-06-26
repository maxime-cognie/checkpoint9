#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <cmath>
#include <cstddef>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

using GoToLoading = attach_shelf::srv::GoToLoading;
using namespace std::chrono_literals;

class ApproachServer : public rclcpp::Node {
public:
  ApproachServer()
      : Node("approach_service_server_node"), cart_detected_(false),
        publish_tf_(false) {

    srv_cbg_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    approach_srv_ = this->create_service<GoToLoading>(
        "approach_shelf",
        std::bind(&ApproachServer::srv_callback, this, std::placeholders::_1,
                  std::placeholders::_2),
        rmw_qos_profile_services_default, srv_cbg_);

    scan_sub_cbg_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options_1;
    options_1.callback_group = scan_sub_cbg_;

    rclcpp::QoS qos_profile_scan_sub(10);
    qos_profile_scan_sub.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", qos_profile_scan_sub,
        std::bind(&ApproachServer::scan_callback, this, std::placeholders::_1),
        options_1);

    tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    transform_stamped_.header.frame_id = "robot_front_laser_link";
    transform_stamped_.child_frame_id = "cart_frame";

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 1);
  }

private:
  void srv_callback(const GoToLoading::Request::SharedPtr request,
                    GoToLoading::Response::SharedPtr response) {
    publish_tf_ = true;

    if (cart_detected_ && request->attach_to_shelf) {
        RCLCPP_INFO(this->get_logger(),
                  "Final approach has been requested.\n"
                  "The robot is moving until it reaches the shelf");
        std::this_thread::sleep_for(100ms);
        geometry_msgs::msg::TransformStamped t;
        std::string origin_frame = "robot_base_link";
        std::string dest_frame = "cart_frame";

        geometry_msgs::msg::Twist cmd_vel;

        while (true) {
        try {
          t = tf_buffer_->lookupTransform(origin_frame, dest_frame,
                                          tf2::TimePointZero, 1s);
        } catch (const tf2::TransformException &ex) {
          RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                      origin_frame.c_str(), dest_frame.c_str(), ex.what());
          return;
        }

        double error_distance;
        double error_yaw;

        error_distance =
            sqrt(t.transform.translation.x * t.transform.translation.x +
                 t.transform.translation.y * t.transform.translation.y);

        error_yaw = atan2(t.transform.translation.y, t.transform.translation.x);

        const double kp_yaw = 2.0 / M_PI;

        cmd_vel.linear.x = 0.2;
        cmd_vel.angular.z = kp_yaw * error_yaw;

        if (std::abs(error_distance) < 0.02) {
          cmd_vel.angular.z = 0.0;
          publisher_->publish(cmd_vel);
          std::this_thread::sleep_for(2500ms);
          break;
        }
        publisher_->publish(cmd_vel);

        std::this_thread::sleep_for(100ms);
      }
      cmd_vel.linear.x = 0.0;
      publisher_->publish(cmd_vel);

      response->complete = true;
      RCLCPP_INFO(this->get_logger(), "Final appraoch done the robot is underneath the shelf.");
    } else {
      response->complete = false;
      RCLCPP_WARN(this->get_logger(), "The cart is not detected finale approach canceled.");
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    std::vector<std::pair<int, int>> intensity_index;
    bool consecutif = false;
    int first_elem, last_elem;
    for (size_t i = 0; i < msg->intensities.size(); i++) {
      if (msg->intensities[i] >= 8000) {
        if (!consecutif) {
          first_elem = i;
          consecutif = true;
        }
      } else if (consecutif) {
        last_elem = i - 1;
        consecutif = false;
        intensity_index.emplace_back(std::make_pair(first_elem, last_elem));
      }
    }
    if (intensity_index.size() > 1) {
      cart_detected_ = true;

      std::vector<std::pair<float, float>> wheel_pos;
      for (auto x : intensity_index) {
        wheel_pos.push_back(
            get_median(get_pos(msg, x.first), get_pos(msg, x.second)));
      }

      cart_pos_ = std::make_shared<std::pair<float, float>>(
          get_median(wheel_pos.front(), wheel_pos.back()));
    } else {
      cart_detected_ = false;
    }
    if (publish_tf_) {
      transform_stamped_.header.stamp = this->get_clock()->now();
      transform_stamped_.transform.translation.x = cart_pos_->first;
      transform_stamped_.transform.translation.y = cart_pos_->second;
      transform_stamped_.transform.translation.z = 0.0;

      tf2::Quaternion transform_quaternion;
      transform_quaternion.setRPY(M_PI, 0.0, 0.0);

      transform_stamped_.transform.rotation.x = transform_quaternion.getX();
      transform_stamped_.transform.rotation.y = transform_quaternion.getY();
      transform_stamped_.transform.rotation.z = transform_quaternion.getZ();
      transform_stamped_.transform.rotation.w = transform_quaternion.getW();

      tf_br_->sendTransform(transform_stamped_);
    }
  }

  std::pair<float, float>
  get_pos(const sensor_msgs::msg::LaserScan::SharedPtr msg, const int ind) {
    auto angle = (static_cast<float>(ind) - (msg->intensities.size() / 2)) *
                 msg->angle_increment;
    return std::make_pair(msg->ranges[ind] * std::cos(angle),
                          msg->ranges[ind] * std::sin(angle));
  }

  std::pair<float, float>
  get_median(const std::pair<float, float> &first_pos,
             const std::pair<float, float> &second_pos) {
    return std::make_pair((first_pos.first + second_pos.first) / 2,
                          (first_pos.second + second_pos.second) / 2);
  }

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

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<ApproachServer> approach_server =
      std::make_shared<ApproachServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(approach_server);
  executor.spin();

  return 0;
}