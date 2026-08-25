#include "wheel_leg_ros/controller_node.hpp"

#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <utility>

#include "wheel_leg_ros/conversions.hpp"

namespace wheel_leg_ros {

ControllerNode::ControllerNode(const rclcpp::NodeOptions &options)
    : Node("wheel_leg_controller", options) {
  const auto max_state_age_ms =
      declare_parameter<std::int64_t>("max_state_age_ms", 100);
  if (max_state_age_ms <= 0) {
    throw std::invalid_argument("max_state_age_ms must be positive");
  }
  wheel_leg::ControllerConfig config;
  config.max_state_age_ns =
      static_cast<std::uint64_t>(max_state_age_ms) * 1'000'000U;
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
}

void ControllerNode::onState(
    const wheel_leg_msgs::msg::RobotState::SharedPtr message) {
  const auto now = std::chrono::steady_clock::now().time_since_epoch();
  const auto now_ns = static_cast<std::uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
  const auto result = core_.step(fromRos(*message), now_ns);
  if (!result.accepted()) {
    RCLCPP_WARN(
        get_logger(), "RobotState rejected by Controller Core (status=%d)",
        static_cast<int>(result.status));
    return;
  }
  publisher_->publish(toRos(result.command));
}

}  // namespace wheel_leg_ros
