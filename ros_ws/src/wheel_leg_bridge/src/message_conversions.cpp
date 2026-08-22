#include "wheel_leg_bridge/message_conversions.hpp"

#include <cstddef>
#include <utility>

#include <std_msgs/msg/header.hpp>

namespace wheel_leg_bridge {
namespace {

wheel_leg_common::TimePoint ToCommonTime(
    const builtin_interfaces::msg::Time& stamp) {
  wheel_leg_common::TimePoint output;
  output.sec = stamp.sec;
  output.nanosec = stamp.nanosec;
  return output;
}

std_msgs::msg::Header ToRosHeader(const wheel_leg_common::TimePoint& stamp) {
  std_msgs::msg::Header header;
  header.stamp.sec = stamp.sec;
  header.stamp.nanosec = stamp.nanosec;
  return header;
}

}  // namespace

wheel_leg_common::JointStateSample FromRosJointState(
    const sensor_msgs::msg::JointState& msg) {
  wheel_leg_common::JointStateSample sample;
  sample.stamp = ToCommonTime(msg.header.stamp);

  const std::size_t joint_count = msg.name.size();
  sample.joints.reserve(joint_count);
  for (std::size_t i = 0; i < joint_count; ++i) {
    wheel_leg_common::JointSample joint;
    joint.name = msg.name[i];
    joint.position = i < msg.position.size() ? msg.position[i] : 0.0;
    joint.velocity = i < msg.velocity.size() ? msg.velocity[i] : 0.0;
    joint.effort = i < msg.effort.size() ? msg.effort[i] : 0.0;
    sample.joints.push_back(std::move(joint));
  }

  return sample;
}

wheel_leg_common::ImuSample FromRosImu(const sensor_msgs::msg::Imu& msg) {
  wheel_leg_common::ImuSample sample;
  sample.stamp = ToCommonTime(msg.header.stamp);
  sample.frame_id = msg.header.frame_id;
  sample.orientation.x = msg.orientation.x;
  sample.orientation.y = msg.orientation.y;
  sample.orientation.z = msg.orientation.z;
  sample.orientation.w = msg.orientation.w;
  sample.angular_velocity.x = msg.angular_velocity.x;
  sample.angular_velocity.y = msg.angular_velocity.y;
  sample.angular_velocity.z = msg.angular_velocity.z;
  sample.linear_acceleration.x = msg.linear_acceleration.x;
  sample.linear_acceleration.y = msg.linear_acceleration.y;
  sample.linear_acceleration.z = msg.linear_acceleration.z;
  return sample;
}

sensor_msgs::msg::JointState ToRosJointState(
    const wheel_leg_common::JointStateSample& sample) {
  sensor_msgs::msg::JointState msg;
  msg.header = ToRosHeader(sample.stamp);
  msg.name.reserve(sample.joints.size());
  msg.position.reserve(sample.joints.size());
  msg.velocity.reserve(sample.joints.size());
  msg.effort.reserve(sample.joints.size());

  for (const auto& joint : sample.joints) {
    msg.name.push_back(joint.name);
    msg.position.push_back(joint.position);
    msg.velocity.push_back(joint.velocity);
    msg.effort.push_back(joint.effort);
  }

  return msg;
}

sensor_msgs::msg::Imu ToRosImu(const wheel_leg_common::ImuSample& sample) {
  sensor_msgs::msg::Imu msg;
  msg.header = ToRosHeader(sample.stamp);
  msg.header.frame_id = sample.frame_id;
  msg.orientation.x = sample.orientation.x;
  msg.orientation.y = sample.orientation.y;
  msg.orientation.z = sample.orientation.z;
  msg.orientation.w = sample.orientation.w;
  msg.angular_velocity.x = sample.angular_velocity.x;
  msg.angular_velocity.y = sample.angular_velocity.y;
  msg.angular_velocity.z = sample.angular_velocity.z;
  msg.linear_acceleration.x = sample.linear_acceleration.x;
  msg.linear_acceleration.y = sample.linear_acceleration.y;
  msg.linear_acceleration.z = sample.linear_acceleration.z;
  return msg;
}

wheel_leg_common::ControlCommand FromRosJointCommand(
    const wheel_leg_msgs::msg::JointCommand& msg) {
  wheel_leg_common::ControlCommand command;
  command.stamp = ToCommonTime(msg.header.stamp);
  command.joint_efforts.reserve(msg.joint_names.size());
  for (std::size_t i = 0; i < msg.joint_names.size(); ++i) {
    wheel_leg_common::JointEffortCommand joint_effort;
    joint_effort.joint_name = msg.joint_names[i];
    joint_effort.effort = i < msg.efforts.size() ? msg.efforts[i] : 0.0;
    command.joint_efforts.push_back(std::move(joint_effort));
  }
  return command;
}

wheel_leg_msgs::msg::JointCommand ToRosJointCommand(
    const wheel_leg_common::ControlCommand& command) {
  wheel_leg_msgs::msg::JointCommand msg;
  msg.header = ToRosHeader(command.stamp);
  msg.joint_names.reserve(command.joint_efforts.size());
  msg.efforts.reserve(command.joint_efforts.size());
  for (const auto& joint_effort : command.joint_efforts) {
    msg.joint_names.push_back(joint_effort.joint_name);
    msg.efforts.push_back(joint_effort.effort);
  }
  return msg;
}

}  // namespace wheel_leg_bridge
