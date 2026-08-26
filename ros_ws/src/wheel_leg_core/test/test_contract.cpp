#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>

#include "wheel_leg_core/controller_core.hpp"
#include "wheel_leg_core/legacy_simulink.hpp"

int main() {
  using namespace wheel_leg;

  static_assert(kJointNames[0] == "left_hip");
  static_assert(kJointNames[5] == "right_wheel");
  const Vector3 canonical_vector{1.0, 2.0, 3.0};
  const auto legacy_vector =
      canonicalFluToLegacyForwardRightUp(canonical_vector);
  assert((legacy_vector == Vector3{1.0, -2.0, 3.0}));
  assert(legacyForwardRightUpToCanonicalFlu(legacy_vector) == canonical_vector);

  RobotState state;
  state.sample_time_ns = 1'000'000'000;
  assert(validateRobotState(state) == ValidationError::kNone);

  ControllerCore core;
  assert(core.configure({1.0e-6}));
  const auto first = core.step(state);
  assert(first.accepted());
  assert(first.dt_s == 0.0);
  for (const double torque : first.command.joint_torque_nm) {
    assert(torque == 0.0);
  }

  state.sample_time_ns += 5'000'000;
  const auto second = core.step(state);
  assert(second.accepted());
  assert(std::abs(second.dt_s - 0.005) < 1.0e-12);
  assert(core.step(state).status ==
         StepStatus::kNonMonotonicState);

  state.q_n_from_b = {2.0, 0.0, 0.0, 0.0};
  assert(core.step(state).status ==
         StepStatus::kInvalidState);
  state.q_n_from_b = {1.0, 0.0, 0.0, 0.0};
  state.joint_velocity_rad_s[2] =
      std::numeric_limits<double>::quiet_NaN();
  assert(validateRobotState(state) == ValidationError::kNonFinite);
  state.joint_velocity_rad_s[2] = 0.0;
  state.contact_state[1] = static_cast<ContactState>(3);
  assert(validateRobotState(state) == ValidationError::kInvalidContact);
  state.contact_state[1] = ContactState::kUnknown;
  state.q_n_from_b = {-1.0, 0.0, 0.0, 0.0};
  assert(validateRobotState(state) == ValidationError::kNone);

  TorqueCommand invalid_command;
  invalid_command.joint_torque_nm[4] =
      std::numeric_limits<double>::infinity();
  assert(validateTorqueCommand(invalid_command) == ValidationError::kNonFinite);

  core.reset();
  assert(core.step(state).accepted());

  ControllerConfig controlled;
  controlled.mode = ControllerMode::kJointPdGravity;
  controlled.enable_pd = true;
  controlled.enable_gravity = true;
  controlled.initial_reference.position_rad = {
      0.2, -0.1, 0.3, -0.2, 0.1, -0.3};
  controlled.initial_reference.velocity_rad_s = {
      0.1, -0.2, 0.3, -0.1, 0.2, -0.3};
  controlled.kp_nm_per_rad = {2.0, 3.0, 4.0, 2.0, 3.0, 4.0};
  controlled.kd_nm_s_per_rad = {0.5, 0.6, 0.7, 0.5, 0.6, 0.7};
  controlled.torque_limit_nm = {0.25, 10.0, 10.0, 10.0, 10.0, 10.0};
  controlled.gravity_profile = currentNominalGravityProfile();
  ControllerCore controlled_core;
  assert(controlled_core.configure(controlled));
  state.sample_time_ns += 1;
  state.joint_position_rad = {};
  state.joint_velocity_rad_s = {};
  const auto controlled_result = controlled_core.step(state);
  assert(controlled_result.accepted());
  assert(controlled_result.saturated[0]);
  assert(std::abs(controlled_result.command.joint_torque_nm[0]) == 0.25);
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    const double expected_pd =
        controlled.kp_nm_per_rad[joint] *
            controlled.initial_reference.position_rad[joint] +
        controlled.kd_nm_s_per_rad[joint] *
            controlled.initial_reference.velocity_rad_s[joint];
    assert(std::abs(controlled_result.tau_pd_nm[joint] - expected_pd) < 1e-15);
    assert(std::abs(
               controlled_result.tau_raw_nm[joint] -
               controlled_result.tau_pd_nm[joint] -
               controlled_result.tau_gravity_nm[joint]) < 1e-15);
  }
  assert(std::abs(controlled_result.tau_gravity_nm[2]) > 1e-6);
  assert(std::abs(controlled_result.tau_gravity_nm[5]) > 1e-6);

  JointReference updated;
  updated.position_rad.fill(0.4);
  assert(controlled_core.setReference(updated));
  state.sample_time_ns += 1;
  const auto updated_result = controlled_core.step(state);
  assert(std::abs(updated_result.tau_pd_nm[1] - 1.2) < 1e-15);
  controlled_core.reset();
  state.sample_time_ns += 1;
  const auto reset_result = controlled_core.step(state);
  assert(std::abs(
             reset_result.tau_pd_nm[1] -
             (3.0 * -0.1 + 0.6 * -0.2)) < 1e-15);

  JointReference invalid_reference;
  invalid_reference.position_rad[0] =
      std::numeric_limits<double>::quiet_NaN();
  assert(!controlled_core.setReference(invalid_reference));
  controlled.kp_nm_per_rad[0] = -1.0;
  assert(!controlled_core.configure(controlled));

  ControllerConfig standing;
  standing.mode = ControllerMode::kSimpleStanding;
  standing.initial_reference.position_rad = {
      -0.96, 1.64, 0.0, -0.98, 1.64, 0.0};
  standing.kp_nm_per_rad = {8.0, 8.0, 0.0, 8.0, 8.0, 0.0};
  standing.kd_nm_s_per_rad = {1.0, 1.0, 0.0, 1.0, 1.0, 0.0};
  standing.torque_limit_nm = {10.0, 10.0, 2.0, 10.0, 10.0, 2.0};
  standing.simple_standing.support_torque_nm = {
      -0.15, -1.95, 0.0, 0.15, -4.42, 0.0};
  standing.simple_standing.gain = {2.0, 3.0, 40.0, 1.0};
  ControllerCore standing_core;
  assert(standing_core.configure(standing));
  RobotState standing_state;
  standing_state.sample_time_ns = 10'000'000;
  standing_state.base_position_n_m = {1.0, 0.0, 0.5};
  standing_state.base_linear_velocity_n_m_s[0] = 0.1;
  standing_state.q_n_from_b = {
      std::cos(0.01 / 2.0), 0.0, std::sin(0.01 / 2.0), 0.0};
  standing_state.base_angular_velocity_n_rad_s[1] = 0.2;
  standing_state.joint_position_rad = standing.initial_reference.position_rad;
  standing_state.contact_state = {
      ContactState::kContact, ContactState::kContact};
  const auto standing_first = standing_core.step(standing_state);
  assert(standing_first.accepted());
  const double expected_wheel = -(3.0 * 0.1 + 40.0 * 0.01 + 1.0 * 0.2);
  assert(std::abs(standing_first.command.joint_torque_nm[2] - expected_wheel) < 1e-12);
  assert(standing_first.command.joint_torque_nm[2] ==
         standing_first.command.joint_torque_nm[5]);
  assert(standing_first.command.joint_torque_nm[0] == -0.15);
  standing_state.sample_time_ns += 10'000'000;
  standing_state.joint_position_rad[0] -= 0.01;
  const auto standing_second = standing_core.step(standing_state);
  assert(standing_second.accepted());
  assert(std::abs(standing_second.command.joint_torque_nm[0] + 0.07) < 1e-12);
  standing_state.sample_time_ns += 10'000'000;
  standing_state.contact_state[0] = ContactState::kNoContact;
  const auto standing_trip = standing_core.step(standing_state);
  assert(standing_trip.status == StepStatus::kSafetyLatched);
  assert(standing_trip.safety_latched);
  for (const double torque : standing_trip.command.joint_torque_nm) {
    assert(torque == 0.0);
  }
  standing_state.sample_time_ns += 10'000'000;
  standing_state.contact_state[0] = ContactState::kContact;
  assert(standing_core.step(standing_state).status == StepStatus::kSafetyLatched);
  standing_core.reset();
  standing_state.sample_time_ns += 10'000'000;
  assert(standing_core.step(standing_state).accepted());
}
