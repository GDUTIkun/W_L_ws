#include "wheel_leg_ros/conversions.hpp"

namespace wheel_leg_ros {

wheel_leg::RobotState fromRos(const wheel_leg_msgs::msg::RobotState &message) {
  wheel_leg::RobotState state;
  state.sample_time_ns = message.sample_time_ns;
  state.base_position_n_m = {
      message.base_position_n_m.x,
      message.base_position_n_m.y,
      message.base_position_n_m.z};
  state.q_n_from_b = {
      message.q_n_from_b.w,
      message.q_n_from_b.x,
      message.q_n_from_b.y,
      message.q_n_from_b.z};
  state.base_linear_velocity_n_m_s = {
      message.base_linear_velocity_n_m_s.x,
      message.base_linear_velocity_n_m_s.y,
      message.base_linear_velocity_n_m_s.z};
  state.base_angular_velocity_n_rad_s = {
      message.base_angular_velocity_n_rad_s.x,
      message.base_angular_velocity_n_rad_s.y,
      message.base_angular_velocity_n_rad_s.z};
  state.joint_position_rad = message.joint_position_rad;
  state.joint_velocity_rad_s = message.joint_velocity_rad_s;
  for (std::size_t index = 0; index < state.contact_state.size(); ++index) {
    state.contact_state[index] =
        static_cast<wheel_leg::ContactState>(message.contact_state[index]);
  }
  return state;
}

wheel_leg_msgs::msg::RobotState toRos(const wheel_leg::RobotState &state) {
  wheel_leg_msgs::msg::RobotState message;
  message.sample_time_ns = state.sample_time_ns;
  message.base_position_n_m.x = state.base_position_n_m[0];
  message.base_position_n_m.y = state.base_position_n_m[1];
  message.base_position_n_m.z = state.base_position_n_m[2];
  message.q_n_from_b.w = state.q_n_from_b[0];
  message.q_n_from_b.x = state.q_n_from_b[1];
  message.q_n_from_b.y = state.q_n_from_b[2];
  message.q_n_from_b.z = state.q_n_from_b[3];
  message.base_linear_velocity_n_m_s.x =
      state.base_linear_velocity_n_m_s[0];
  message.base_linear_velocity_n_m_s.y =
      state.base_linear_velocity_n_m_s[1];
  message.base_linear_velocity_n_m_s.z =
      state.base_linear_velocity_n_m_s[2];
  message.base_angular_velocity_n_rad_s.x =
      state.base_angular_velocity_n_rad_s[0];
  message.base_angular_velocity_n_rad_s.y =
      state.base_angular_velocity_n_rad_s[1];
  message.base_angular_velocity_n_rad_s.z =
      state.base_angular_velocity_n_rad_s[2];
  message.joint_position_rad = state.joint_position_rad;
  message.joint_velocity_rad_s = state.joint_velocity_rad_s;
  for (std::size_t index = 0; index < state.contact_state.size(); ++index) {
    message.contact_state[index] =
        static_cast<std::uint8_t>(state.contact_state[index]);
  }
  return message;
}

wheel_leg::TorqueCommand fromRos(
    const wheel_leg_msgs::msg::TorqueCommand &message) {
  wheel_leg::TorqueCommand command;
  command.source_sample_time_ns = message.source_sample_time_ns;
  command.joint_torque_nm = message.joint_torque_nm;
  return command;
}

wheel_leg_msgs::msg::TorqueCommand toRos(
    const wheel_leg::TorqueCommand &command) {
  wheel_leg_msgs::msg::TorqueCommand message;
  message.source_sample_time_ns = command.source_sample_time_ns;
  message.joint_torque_nm = command.joint_torque_nm;
  return message;
}

}  // namespace wheel_leg_ros
