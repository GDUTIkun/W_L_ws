#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

#include "wheel_leg_core/controller_core.hpp"

namespace {

wheel_leg::RobotState readState() {
  std::ifstream input(WBC_PROBLEM_GOLDEN_PATH);
  std::string header;
  std::string case_id;
  int count = 0;
  assert(input >> header >> count >> case_id);
  wheel_leg::RobotState state;
  for (double &value : state.base_position_n_m) assert(input >> value);
  for (double &value : state.q_n_from_b) assert(input >> value);
  for (double &value : state.base_linear_velocity_n_m_s) assert(input >> value);
  for (double &value : state.base_angular_velocity_n_rad_s) assert(input >> value);
  for (double &value : state.joint_position_rad) assert(input >> value);
  for (double &value : state.joint_velocity_rad_s) assert(input >> value);
  state.contact_state = {wheel_leg::ContactState::kContact,
                         wheel_leg::ContactState::kContact};
  state.sample_time_ns = 1000000000;
  return state;
}

wheel_leg::ControllerConfig config() {
  wheel_leg::ControllerConfig value;
  value.mode = wheel_leg::ControllerMode::kPhase27MinimalNmpcWbc;
  value.torque_limit_nm = {10.0, 10.0, 2.0, 10.0, 10.0, 2.0};
  value.weighted_wbc = wheel_leg::currentNominalWeightedWbcConfig();
  return value;
}

void assertZero(const wheel_leg::TorqueCommand &command) {
  for (double value : command.joint_torque_nm) assert(value == 0.0);
}

wheel_leg::StepResult acceptedColdStep(
    wheel_leg::ControllerCore &controller,
    const wheel_leg::ControllerConfig &controller_config,
    const wheel_leg::RobotState &state) {
  wheel_leg::StepResult result;
  for (int attempt = 0; attempt < 5; ++attempt) {
    result = controller.step(state);
    if (result.accepted()) return result;
    const bool host_deadline_only =
        result.phase27_nmpc_result.ok() &&
        result.weighted_wbc_status ==
            wheel_leg::WeightedWbcController::Status::kOk &&
        result.phase27_nmpc_wbc_total_time_s >
            controller_config.phase27_nmpc.deadline_s;
    if (!host_deadline_only) return result;
    assert(controller.configure(controller_config));
  }
  return result;
}

}  // namespace

int main() {
  auto state = readState();
  wheel_leg::ControllerCore controller;
  assert(controller.configure(config()));
  assert(controller.setPhase27MotionReference(0.20, 0.08));
  assert(!controller.setPhase27MotionReference(0.201, 0.0));
  assert(!controller.setPhase27MotionReference(0.0, 0.081));
  assert(controller.setPhase27MotionReference(0.0, 0.0));
  const auto first = acceptedColdStep(controller, config(), state);
  if (!first.accepted()) {
    std::cerr << "first status=" << static_cast<int>(first.status)
              << " latched=" << first.safety_latched
              << " nmpc_status="
              << static_cast<int>(first.phase27_nmpc_result.status)
              << " acados=" << first.phase27_nmpc_result.acados_status
              << " nmpc_time=" << first.phase27_nmpc_result.solve_time_s
              << " wbc_status=" << static_cast<int>(first.weighted_wbc_status)
              << " wbc_model="
              << static_cast<int>(first.weighted_wbc_model_status)
              << " wbc_solver="
              << static_cast<int>(first.weighted_wbc_solver_status)
              << " wbc_hard=" << first.weighted_wbc_hard_violation
              << " total_time=" << first.phase27_nmpc_wbc_total_time_s
              << '\n';
  }
  assert(first.accepted());
  assert(first.phase27_nmpc_active && first.phase27_nmpc_update_tick);
  assert(first.phase27_nmpc_wrench_age_ticks == 0);
  assert(first.phase27_nmpc_result.ok());
  assert(first.weighted_wbc_active);
  assert(first.weighted_wbc_status ==
         wheel_leg::WeightedWbcController::Status::kOk);
  assert(first.phase27_nmpc_wbc_total_time_s <= 0.01);
  assert(first.phase27_wheel_reference.common_position_m != 0.0);
  assert(first.weighted_wbc_task_normalized_squared_cost[
             static_cast<std::size_t>(
                 wheel_leg::WeightedWbcController::Task::kBaseX)] == 0.0);

  state.sample_time_ns += 10000000;
  const auto held = controller.step(state);
  assert(held.accepted());
  assert(!held.phase27_nmpc_update_tick);
  assert(held.phase27_nmpc_wrench_age_ticks == 1);
  assert((first.phase27_nmpc_result.interaction_wrench_flu -
          held.phase27_nmpc_result.interaction_wrench_flu)
             .cwiseAbs().maxCoeff() == 0.0);
  state.sample_time_ns += 10000000;
  const auto next = controller.step(state);
  assert(next.accepted() && next.phase27_nmpc_update_tick);
  assert(next.phase27_nmpc_wrench_age_ticks == 0);

  controller.reset();
  state = readState();
  const auto replay = acceptedColdStep(controller, config(), state);
  assert(replay.accepted());
  for (std::size_t joint = 0; joint < wheel_leg::kJointCount; ++joint) {
    assert(first.command.joint_torque_nm[joint] ==
           replay.command.joint_torque_nm[joint]);
  }

  for (const auto fault : {wheel_leg::NmpcFaultInjection::kSolverFailure,
                           wheel_leg::NmpcFaultInjection::kLate,
                           wheel_leg::NmpcFaultInjection::kStale,
                           wheel_leg::NmpcFaultInjection::kNonFinite}) {
    auto fault_config = config();
    fault_config.phase27_nmpc.fault_injection = fault;
    fault_config.phase27_nmpc.fault_control_tick = 0;
    wheel_leg::ControllerCore fault_controller;
    assert(fault_controller.configure(fault_config));
    const auto rejected = fault_controller.step(readState());
    assert(rejected.status == wheel_leg::StepStatus::kSafetyLatched);
    assert(rejected.safety_latched);
    assertZero(rejected.command);
    auto later = readState();
    later.sample_time_ns += 10000000;
    const auto latched = fault_controller.step(later);
    assert(latched.status == wheel_leg::StepStatus::kSafetyLatched);
    assertZero(latched.command);
    fault_controller.reset();
    fault_config.phase27_nmpc.fault_injection =
        wheel_leg::NmpcFaultInjection::kNone;
    assert(fault_controller.configure(fault_config));
    assert(acceptedColdStep(fault_controller, fault_config, readState()).accepted());
  }

  auto invalid = config();
  invalid.phase27_nmpc.longitudinal_velocity_m_s = 0.201;
  assert(!controller.configure(invalid));
  invalid = config();
  invalid.phase27_nmpc.yaw_rate_rad_s = 0.081;
  assert(!controller.configure(invalid));
  invalid = config();
  invalid.phase27_nmpc.target_common_position_offset_m =
      std::numeric_limits<double>::quiet_NaN();
  assert(!controller.configure(invalid));
  return 0;
}
