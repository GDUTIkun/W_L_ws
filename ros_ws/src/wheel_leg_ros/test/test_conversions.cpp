#include <cmath>

#include "gtest/gtest.h"
#include "wheel_leg_ros/conversions.hpp"

TEST(Conversions, RobotStateRoundTripPreservesCanonicalFields) {
  wheel_leg::RobotState state;
  state.sample_time_ns = 42;
  state.base_position_n_m = {1.0, 2.0, 3.0};
  state.q_n_from_b = {0.5, -0.5, 0.5, -0.5};
  state.base_linear_velocity_n_m_s = {4.0, 5.0, 6.0};
  state.base_angular_velocity_n_rad_s = {7.0, 8.0, 9.0};
  state.joint_position_rad = {1, 2, 3, 4, 5, 6};
  state.joint_velocity_rad_s = {-1, -2, -3, -4, -5, -6};
  state.contact_state = {
      wheel_leg::ContactState::kContact,
      wheel_leg::ContactState::kNoContact};

  const auto message = wheel_leg_ros::toRos(state);
  EXPECT_DOUBLE_EQ(message.q_n_from_b.x, -0.5);
  EXPECT_DOUBLE_EQ(message.q_n_from_b.y, 0.5);
  EXPECT_DOUBLE_EQ(message.q_n_from_b.z, -0.5);
  EXPECT_DOUBLE_EQ(message.q_n_from_b.w, 0.5);

  const auto round_trip = wheel_leg_ros::fromRos(message);
  EXPECT_EQ(round_trip.sample_time_ns, state.sample_time_ns);
  EXPECT_EQ(round_trip.base_position_n_m, state.base_position_n_m);
  EXPECT_EQ(round_trip.q_n_from_b, state.q_n_from_b);
  EXPECT_EQ(round_trip.joint_position_rad, state.joint_position_rad);
  EXPECT_EQ(round_trip.joint_velocity_rad_s, state.joint_velocity_rad_s);
  EXPECT_EQ(round_trip.contact_state, state.contact_state);
}

TEST(Conversions, TorqueCommandRoundTripPreservesJointOrder) {
  wheel_leg::TorqueCommand command;
  command.source_sample_time_ns = 99;
  command.joint_torque_nm = {1, 2, 3, 4, 5, 6};
  const auto round_trip = wheel_leg_ros::fromRos(wheel_leg_ros::toRos(command));
  EXPECT_EQ(round_trip.source_sample_time_ns, command.source_sample_time_ns);
  EXPECT_EQ(round_trip.joint_torque_nm, command.joint_torque_nm);
}
