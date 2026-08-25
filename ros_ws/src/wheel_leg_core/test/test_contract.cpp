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
  assert(core.configure({100'000'000, 1.0e-6}));
  const auto first = core.step(state, 1'010'000'000);
  assert(first.accepted());
  assert(first.dt_s == 0.0);
  for (const double torque : first.command.joint_torque_nm) {
    assert(torque == 0.0);
  }

  state.sample_time_ns += 5'000'000;
  const auto second = core.step(state, 1'015'000'000);
  assert(second.accepted());
  assert(std::abs(second.dt_s - 0.005) < 1.0e-12);
  assert(core.step(state, 1'015'000'000).status ==
         StepStatus::kNonMonotonicState);

  state.sample_time_ns += 1;
  assert(core.step(state, state.sample_time_ns - 1).status ==
         StepStatus::kFutureState);
  assert(core.step(state, state.sample_time_ns + 100'000'001).status ==
         StepStatus::kStaleState);

  state.q_n_from_b = {2.0, 0.0, 0.0, 0.0};
  assert(core.step(state, state.sample_time_ns).status ==
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
  assert(core.step(state, state.sample_time_ns).accepted());
}
