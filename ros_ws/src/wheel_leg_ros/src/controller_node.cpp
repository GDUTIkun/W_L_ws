#include "wheel_leg_ros/controller_node.hpp"

#include <stdexcept>
#include <utility>

#include "wheel_leg_ros/conversions.hpp"

namespace wheel_leg_ros {

ControllerNode::ControllerNode(const rclcpp::NodeOptions &options)
    : Node("wheel_leg_controller", options) {
  wheel_leg::ControllerConfig config;
  if (!core_.configure(config)) {
    throw std::runtime_error("invalid Controller Core configuration");
  }

  publisher_ = create_publisher<wheel_leg_msgs::msg::TorqueCommand>(
      "torque_command", 10);
  subscription_ = create_subscription<wheel_leg_msgs::msg::RobotState>(
      "robot_state", rclcpp::SensorDataQoS(),
      [this](wheel_leg_msgs::msg::RobotState::SharedPtr message) {
        onState(std::move(message));
      });
  reset_service_ = create_service<std_srvs::srv::Trigger>(
      "reset_controller",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        core_.reset();
        response->success = true;
        response->message = "Controller sample history reset";
      });
}

void ControllerNode::onState(
    const wheel_leg_msgs::msg::RobotState::SharedPtr message) {
  const auto result = core_.step(fromRos(*message));
  if (!result.accepted()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "RobotState rejected by Controller Core (status=%d)",
        static_cast<int>(result.status));
    return;
  }
  publisher_->publish(toRos(result.command));
}

}  // namespace wheel_leg_ros
