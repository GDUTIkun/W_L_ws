#ifndef WHEEL_LEG_BRIDGE__MESSAGE_CONVERSIONS_HPP_
#define WHEEL_LEG_BRIDGE__MESSAGE_CONVERSIONS_HPP_

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <wheel_leg_common/types.hpp>
#include <wheel_leg_msgs/msg/joint_command.hpp>

namespace wheel_leg_bridge {

wheel_leg_common::JointStateSample FromRosJointState(
    const sensor_msgs::msg::JointState& msg);

wheel_leg_common::ImuSample FromRosImu(const sensor_msgs::msg::Imu& msg);

sensor_msgs::msg::JointState ToRosJointState(
    const wheel_leg_common::JointStateSample& sample);

sensor_msgs::msg::Imu ToRosImu(const wheel_leg_common::ImuSample& sample);

wheel_leg_common::ControlCommand FromRosJointCommand(
    const wheel_leg_msgs::msg::JointCommand& msg);

wheel_leg_msgs::msg::JointCommand ToRosJointCommand(
    const wheel_leg_common::ControlCommand& command);

}  // namespace wheel_leg_bridge

#endif  // WHEEL_LEG_BRIDGE__MESSAGE_CONVERSIONS_HPP_
