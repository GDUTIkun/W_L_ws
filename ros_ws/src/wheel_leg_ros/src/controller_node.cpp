#include "wheel_leg_ros/controller_node.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "wheel_leg_ros/conversions.hpp"

namespace wheel_leg_ros {
namespace {

wheel_leg::JointVector jointParameter(
    rclcpp::Node &node, const std::string &name) {
  const auto values = node.declare_parameter<std::vector<double>>(
      name, std::vector<double>(wheel_leg::kJointCount, 0.0));
  if (values.size() != wheel_leg::kJointCount) {
    throw std::runtime_error(name + " must contain six values");
  }
  wheel_leg::JointVector result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

std::array<double, 3> tripleParameter(
    rclcpp::Node &node, const std::string &name) {
  const auto values = node.declare_parameter<std::vector<double>>(
      name, std::vector<double>(3, 0.0));
  if (values.size() != 3) {
    throw std::runtime_error(name + " must contain three values");
  }
  std::array<double, 3> result{};
  std::copy(values.begin(), values.end(), result.begin());
  return result;
}

void setCoefficients(
    std::array<wheel_leg::GravityHarmonic, 3> &harmonics,
    const std::array<double, 3> &values, bool sine) {
  for (std::size_t index = 0; index < harmonics.size(); ++index) {
    if (sine) {
      harmonics[index].sin_torque_nm = values[index];
    } else {
      harmonics[index].cos_torque_nm = values[index];
    }
  }
}

}  // namespace

ControllerNode::ControllerNode(const rclcpp::NodeOptions &options)
    : Node("wheel_leg_controller", options) {
  wheel_leg::ControllerConfig config;
  const auto mode = declare_parameter<std::string>("controller.mode", "zero");
  if (mode == "weighted_wbc") {
    config = wheel_leg::currentNominalWeightedWbcControllerConfig();
  } else if (mode == "joint_pd_gravity") {
    config.mode = wheel_leg::ControllerMode::kJointPdGravity;
    config.enable_pd = declare_parameter<bool>("controller.enable_pd", true);
    config.enable_gravity =
        declare_parameter<bool>("controller.enable_gravity", true);
    config.initial_reference.position_rad =
        jointParameter(*this, "controller.reference_position_rad");
    config.initial_reference.velocity_rad_s =
        jointParameter(*this, "controller.reference_velocity_rad_s");
    config.kp_nm_per_rad = jointParameter(*this, "controller.kp_nm_per_rad");
    config.kd_nm_s_per_rad =
        jointParameter(*this, "controller.kd_nm_s_per_rad");
    config.torque_limit_nm =
        jointParameter(*this, "controller.torque_limit_nm");
    const auto gravity_profile = declare_parameter<std::string>(
        "controller.gravity_profile", "current_nominal");
    if (gravity_profile == "current_nominal") {
      config.gravity_profile = wheel_leg::currentNominalGravityProfile();
    } else if (gravity_profile == "configured_harmonics") {
      config.gravity_profile = wheel_leg::currentNominalGravityProfile();
      const auto offsets =
          jointParameter(*this, "controller.gravity_canonical_offset_rad");
      std::copy_n(
          offsets.begin(), 3,
          config.gravity_profile.left.canonical_offset_rad.begin());
      std::copy_n(
          offsets.begin() + 3, 3,
          config.gravity_profile.right.canonical_offset_rad.begin());
      setCoefficients(
          config.gravity_profile.left.harmonics,
          tripleParameter(*this, "controller.gravity_left_sin_torque_nm"),
          true);
      setCoefficients(
          config.gravity_profile.left.harmonics,
          tripleParameter(*this, "controller.gravity_left_cos_torque_nm"),
          false);
      setCoefficients(
          config.gravity_profile.right.harmonics,
          tripleParameter(*this, "controller.gravity_right_sin_torque_nm"),
          true);
      setCoefficients(
          config.gravity_profile.right.harmonics,
          tripleParameter(*this, "controller.gravity_right_cos_torque_nm"),
          false);
    } else {
      throw std::runtime_error("unknown controller.gravity_profile");
    }
  } else if (mode != "zero") {
    throw std::runtime_error(
        "controller.mode must be zero, joint_pd_gravity, or weighted_wbc");
  }
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
