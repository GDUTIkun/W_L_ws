#pragma once

#include "wheel_leg_core/types.hpp"
#include "wheel_leg_msgs/msg/robot_state.hpp"
#include "wheel_leg_msgs/msg/torque_command.hpp"

namespace wheel_leg_ros {

wheel_leg::RobotState fromRos(const wheel_leg_msgs::msg::RobotState &message);
wheel_leg_msgs::msg::RobotState toRos(const wheel_leg::RobotState &state);
wheel_leg::TorqueCommand fromRos(
    const wheel_leg_msgs::msg::TorqueCommand &message);
wheel_leg_msgs::msg::TorqueCommand toRos(
    const wheel_leg::TorqueCommand &command);

}  // namespace wheel_leg_ros
