#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "wheel_leg_core/controller_core.hpp"
#include "wheel_leg_msgs/msg/robot_state.hpp"
#include "wheel_leg_msgs/msg/torque_command.hpp"

namespace wheel_leg_ros {

class ControllerNode final : public rclcpp::Node {
 public:
  explicit ControllerNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

 private:
  void onState(const wheel_leg_msgs::msg::RobotState::SharedPtr message);

  wheel_leg::ControllerCore core_;
  rclcpp::Subscription<wheel_leg_msgs::msg::RobotState>::SharedPtr subscription_;
  rclcpp::Publisher<wheel_leg_msgs::msg::TorqueCommand>::SharedPtr publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
};

}  // namespace wheel_leg_ros
