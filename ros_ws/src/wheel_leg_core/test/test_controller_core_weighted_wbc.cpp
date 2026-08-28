#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <string>

#include "wheel_leg_core/controller_core.hpp"

namespace {

wheel_leg::RobotState readState(std::istream &input) {
  wheel_leg::RobotState state;
  for (double &value : state.base_position_n_m) assert(input >> value);
  for (double &value : state.q_n_from_b) assert(input >> value);
  for (double &value : state.base_linear_velocity_n_m_s) assert(input >> value);
  for (double &value : state.base_angular_velocity_n_rad_s) assert(input >> value);
  for (double &value : state.joint_position_rad) assert(input >> value);
  for (double &value : state.joint_velocity_rad_s) assert(input >> value);
  state.contact_state = {wheel_leg::ContactState::kContact,
                         wheel_leg::ContactState::kContact};
  return state;
}

wheel_leg::WbcReference readReference(std::istream &input) {
  wheel_leg::WbcReference reference;
  assert(input >> reference.base_x_acceleration_m_s2);
  assert(input >> reference.base_height_acceleration_m_s2);
  for (Eigen::Index index = 0; index < 3; ++index)
    assert(input >> reference.orientation_acceleration_rad_s2[index]);
  for (Eigen::Index index = 0; index < 4; ++index)
    assert(input >> reference.leg_acceleration_rad_s2[index]);
  for (Eigen::Index index = 0; index < 12; ++index)
    assert(input >> reference.interaction_wrench_flu[index]);
  return reference;
}

void assertZero(const wheel_leg::StepResult &result) {
  for (const double torque : result.command.joint_torque_nm) assert(torque == 0.0);
}

wheel_leg::ControllerConfig weightedConfig(double torque_limit_nm = 100.0) {
  wheel_leg::ControllerConfig config;
  config.mode = wheel_leg::ControllerMode::kWeightedWbc;
  config.torque_limit_nm.fill(torque_limit_nm);
  config.weighted_wbc = wheel_leg::currentNominalWeightedWbcConfig();
  return config;
}

wheel_leg::QuaternionWxyz axisAngleQuaternion(
    std::size_t axis, double angle_rad) {
  wheel_leg::QuaternionWxyz quaternion{std::cos(angle_rad * 0.5), 0.0, 0.0,
                                       0.0};
  quaternion[axis + 1] = std::sin(angle_rad * 0.5);
  return quaternion;
}

template <typename Mutate>
void assertSafetyGate(const wheel_leg::RobotState &equilibrium, Mutate mutate) {
  wheel_leg::ControllerCore core;
  assert(core.configure(weightedConfig()));
  auto anchor = equilibrium;
  anchor.sample_time_ns = 1'000'000'000;
  assert(core.step(anchor).accepted());
  auto violating = anchor;
  violating.sample_time_ns += 10'000'000;
  mutate(violating);
  const auto rejected = core.step(violating);
  assert(rejected.status == wheel_leg::StepStatus::kSafetyLatched);
  assert(rejected.safety_latched);
  assert(rejected.weighted_wbc_active);
  assert(rejected.command.source_sample_time_ns == violating.sample_time_ns);
  assertZero(rejected);
  auto later = anchor;
  later.sample_time_ns += 20'000'000;
  const auto latched = core.step(later);
  assert(latched.status == wheel_leg::StepStatus::kSafetyLatched);
  assert(latched.weighted_wbc_active);
  assert(latched.command.source_sample_time_ns == later.sample_time_ns);
  assertZero(latched);
  core.reset();
  const auto recovered = core.step(anchor);
  assert(recovered.accepted());
}

void assertReferenceEqual(
    const wheel_leg::WbcReference &actual,
    const wheel_leg::WbcReference &expected) {
  assert(std::abs(actual.base_x_acceleration_m_s2 -
                  expected.base_x_acceleration_m_s2) <= 1.0e-12);
  assert(std::abs(actual.base_height_acceleration_m_s2 -
                  expected.base_height_acceleration_m_s2) <= 1.0e-12);
  assert((actual.orientation_acceleration_rad_s2 -
          expected.orientation_acceleration_rad_s2)
             .cwiseAbs()
             .maxCoeff() <= 1.0e-12);
  assert((actual.leg_acceleration_rad_s2 - expected.leg_acceleration_rad_s2)
             .cwiseAbs()
             .maxCoeff() <= 1.0e-12);
  assert((actual.interaction_wrench_flu - expected.interaction_wrench_flu)
             .cwiseAbs()
             .maxCoeff() <= 1.0e-12);
}

}  // namespace

int main() {
  std::ifstream input(WBC_PROBLEM_GOLDEN_PATH);
  assert(input);
  std::string header;
  int count = 0;
  assert(input >> header >> count);
  assert(header == "WBC_PROBLEM_GOLDEN_V1" && count == 32);
  std::string case_id;
  assert(input >> case_id);
  auto equilibrium = readState(input);
  const auto golden_reference = readReference(input);
  equilibrium.sample_time_ns = 1'000'000'000;

  wheel_leg::ControllerCore core;
  assert(core.configure(weightedConfig()));
  const auto cold = core.step(equilibrium);
  assert(cold.accepted());
  assert(cold.weighted_wbc_active);
  assert(cold.command.source_sample_time_ns == equilibrium.sample_time_ns);
  assert(cold.dt_s == 0.0);
  assertReferenceEqual(cold.weighted_wbc_reference, golden_reference);

  wheel_leg::WeightedWbcController independent;
  const auto expected = independent.step(equilibrium, golden_reference);
  assert(expected.ok());
  assert(cold.weighted_wbc_status == expected.status);
  assert(cold.weighted_wbc_model_status == expected.model_status);
  assert(cold.weighted_wbc_solver_status == expected.solver_status);
  assert(cold.weighted_wbc_iterations == expected.iterations);
  assert(std::abs(cold.weighted_wbc_hard_violation -
                  expected.hard_violation) <= 1.0e-15);
  assert(std::abs(cold.weighted_wbc_stationarity_residual -
                  expected.stationarity_residual) <= 1.0e-15);
  assert(std::abs(cold.weighted_wbc_primal_residual -
                  expected.primal_residual) <= 1.0e-15);
  assert(std::abs(cold.weighted_wbc_dual_residual -
                  expected.dual_residual) <= 1.0e-15);
  assert(cold.weighted_wbc_model_diagnostics.reconstruction_iterations ==
         expected.model_diagnostics.reconstruction_iterations);
  assert(std::abs(cold.weighted_wbc_maximum_normalized_slack -
                  expected.maximum_normalized_slack) <= 1.0e-15);
  for (std::size_t index = 0; index < 42; ++index) {
    assert(std::abs(cold.weighted_wbc_physical_solution[index] -
                    expected.physical_solution[index]) <= 1.0e-15);
  }
  for (std::size_t task = 0;
       task < wheel_leg::WeightedWbcController::kTaskCount; ++task) {
    assert(std::abs(cold.weighted_wbc_task_max_abs_normalized_residual[task] -
                    expected.task_max_abs_normalized_residual[task]) <= 1.0e-15);
    assert(std::abs(cold.weighted_wbc_task_normalized_squared_cost[task] -
                    expected.task_normalized_squared_cost[task]) <= 1.0e-15);
  }
  for (std::size_t joint = 0; joint < wheel_leg::kJointCount; ++joint) {
    assert(std::abs(cold.command.joint_torque_nm[joint] -
                    expected.torque_nm[joint]) <= 1.0e-12);
  }

  auto warm_state = equilibrium;
  warm_state.sample_time_ns += 10'000'000;
  const auto warm = core.step(warm_state);
  assert(warm.accepted());
  assert(std::abs(warm.dt_s - 0.01) <= 1.0e-15);
  core.reset();
  const auto repeated_cold = core.step(equilibrium);
  assert(repeated_cold.accepted());
  for (std::size_t joint = 0; joint < wheel_leg::kJointCount; ++joint) {
    assert(std::abs(cold.command.joint_torque_nm[joint] -
                    repeated_cold.command.joint_torque_nm[joint]) <= 1.0e-12);
  }

  assertSafetyGate(equilibrium, [](wheel_leg::RobotState &state) {
    state.contact_state[0] = wheel_leg::ContactState::kNoContact;
  });
  assertSafetyGate(equilibrium, [](wheel_leg::RobotState &state) {
    state.sample_time_ns += 10'000'000;
  });
  assertSafetyGate(equilibrium, [](wheel_leg::RobotState &state) {
    state.base_position_n_m[0] += 0.020001;
  });
  assertSafetyGate(equilibrium, [](wheel_leg::RobotState &state) {
    state.base_position_n_m[1] += 0.020001;
  });
  assertSafetyGate(equilibrium, [](wheel_leg::RobotState &state) {
    state.base_position_n_m[2] += 0.010001;
  });
  assertSafetyGate(equilibrium, [](wheel_leg::RobotState &state) {
    state.q_n_from_b = axisAngleQuaternion(0, 0.030001);
  });
  assertSafetyGate(equilibrium, [](wheel_leg::RobotState &state) {
    state.q_n_from_b = axisAngleQuaternion(1, 0.030001);
  });
  assertSafetyGate(equilibrium, [](wheel_leg::RobotState &state) {
    state.q_n_from_b = axisAngleQuaternion(2, 0.050001);
  });

  {
    wheel_leg::ControllerCore invalid_core;
    assert(invalid_core.configure(weightedConfig()));
    auto invalid = equilibrium;
    invalid.q_n_from_b[0] = std::numeric_limits<double>::quiet_NaN();
    const auto rejected = invalid_core.step(invalid);
    assert(rejected.status == wheel_leg::StepStatus::kInvalidState);
    assert(rejected.safety_latched);
    assertZero(rejected);
  }
  {
    wheel_leg::ControllerCore nonmonotonic_core;
    assert(nonmonotonic_core.configure(weightedConfig()));
    assert(nonmonotonic_core.step(equilibrium).accepted());
    const auto rejected = nonmonotonic_core.step(equilibrium);
    assert(rejected.status == wheel_leg::StepStatus::kNonMonotonicState);
    assert(rejected.safety_latched);
    assertZero(rejected);
  }
  {
    wheel_leg::ControllerCore limited_core;
    assert(limited_core.configure(weightedConfig(1.0e-12)));
    const auto rejected = limited_core.step(equilibrium);
    assert(rejected.status == wheel_leg::StepStatus::kSafetyLatched);
    assert(rejected.safety_latched);
    assertZero(rejected);
    assert(rejected.saturated[0]);
  }
  {
    wheel_leg::ControllerCore zero_core;
    wheel_leg::ControllerConfig zero_config;
    zero_config.mode = wheel_leg::ControllerMode::kZero;
    assert(zero_core.configure(zero_config));
    const auto result = zero_core.step(equilibrium);
    assert(result.accepted());
    assertZero(result);
  }
  return 0;
}
