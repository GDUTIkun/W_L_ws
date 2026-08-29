#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "nominal_wbc_profile_data.hpp"
#include "wheel_leg_core/weighted_wbc_controller.hpp"

namespace {

wheel_leg::RobotState readEquilibriumState() {
  std::ifstream input(WBC_PROBLEM_GOLDEN_PATH);
  std::string header;
  int count = 0;
  std::string case_id;
  assert(input >> header >> count >> case_id);
  assert(header == "WBC_PROBLEM_GOLDEN_V1" && count == 32);
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

std::vector<wheel_leg::RobotState> readDynamicStates() {
  std::ifstream input(PHASE27_WHEEL_GOLDEN_PATH);
  std::string header;
  int count = 0;
  assert(input >> header >> count);
  assert(header == "PHASE27_WHEEL_INTERACTION_GOLDEN_V1" && count == 4);
  std::vector<wheel_leg::RobotState> states;
  states.reserve(count);
  for (int sample = 0; sample < count; ++sample) {
    std::string case_id;
    wheel_leg::RobotState state;
    assert(input >> case_id);
    for (double &value : state.base_position_n_m) assert(input >> value);
    for (double &value : state.q_n_from_b) assert(input >> value);
    for (double &value : state.base_linear_velocity_n_m_s) assert(input >> value);
    for (double &value : state.base_angular_velocity_n_rad_s) assert(input >> value);
    for (double &value : state.joint_position_rad) assert(input >> value);
    for (double &value : state.joint_velocity_rad_s) assert(input >> value);
    state.contact_state = {wheel_leg::ContactState::kContact,
                           wheel_leg::ContactState::kContact};
    double ignored = 0.0;
    for (int index = 0; index < 4 + 2 * (6 * 12 + 6 * 6 + 6); ++index) {
      assert(input >> ignored);
    }
    states.push_back(state);
  }
  return states;
}

wheel_leg::WbcReference equilibriumReference() {
  wheel_leg::WbcReference reference;
  reference.interaction_wrench_flu <<
      0.0, 0.0, 27.675229491866027, 0.11327183296816838, 0.0, 0.0,
      0.0, 0.0, 28.714612508133982, 0.11327183296816838, 0.0, 0.0;
  return reference;
}

template <int Rows>
void addTask(wheel_leg::WeightedWbcProblem::Matrix42 &h,
             wheel_leg::WeightedWbcProblem::Vector42 &g,
             const Eigen::Matrix<double, Rows, 42> &physical,
             const Eigen::Matrix<double, Rows, 1> &target,
             const Eigen::Matrix<double, Rows, 1> &scale) {
  auto normalized = physical;
  auto normalized_target = target;
  for (int row = 0; row < Rows; ++row) {
    normalized.row(row) /= scale[row];
    normalized_target[row] /= scale[row];
  }
  h.noalias() += normalized.transpose() * normalized;
  g.noalias() -= normalized.transpose() * normalized_target;
}

}  // namespace

int main() {
  const auto state = readEquilibriumState();
  const auto reference = equilibriumReference();
  const auto model = wheel_leg::NominalWbcModel{}.evaluate(state);
  assert(model.ok());
  wheel_leg::WeightedWbcProblem assembler;
  const auto nominal = assembler.assemble(model, reference);
  const auto minimal = assembler.assemble(
      model, reference, wheel_leg::WeightedWbcProfile::kPhase27Minimal);
  assert(nominal.ok() && minimal.ok());
  assert((nominal.a - minimal.a).cwiseAbs().maxCoeff() == 0.0);
  assert((nominal.lower - minimal.lower).cwiseAbs().maxCoeff() == 0.0);
  assert((nominal.upper - minimal.upper).cwiseAbs().maxCoeff() == 0.0);

  Eigen::Matrix<double, 42, 1> variable_scale;
  for (int index = 0; index < 42; ++index) {
    variable_scale[index] = wheel_leg::phase21_profile::kVariableScale[index];
  }
  const Eigen::DiagonalMatrix<double, 42> transform(variable_scale);
  wheel_leg::WeightedWbcProblem::Matrix42 expected_h =
      wheel_leg::WeightedWbcProblem::Matrix42::Zero();
  wheel_leg::WeightedWbcProblem::Vector42 expected_g =
      wheel_leg::WeightedWbcProblem::Vector42::Zero();
  expected_h.diagonal().head<30>().setConstant(1.0e-6);
  Eigen::Matrix<double, 6, 42> contact =
      Eigen::Matrix<double, 6, 42>::Zero();
  contact.block<3, 12>(0, 0) = model.contact_jacobian[0];
  contact.block<3, 12>(3, 0) = model.contact_jacobian[1];
  contact *= transform;
  Eigen::Matrix<double, 6, 1> contact_target;
  contact_target << -model.contact_bias[0], -model.contact_bias[1];
  addTask<6>(expected_h, expected_g, contact, contact_target,
             Eigen::Matrix<double, 6, 1>::Constant(10.0));
  Eigen::Matrix<double, 12, 42> wrench =
      Eigen::Matrix<double, 12, 42>::Zero();
  Eigen::Matrix<double, 12, 1> target = reference.interaction_wrench_flu;
  for (int side = 0; side < 2; ++side) {
    wrench.block<6, 12>(6 * side, 0) =
        model.interaction_acceleration_map[side];
    wrench.block<6, 6>(6 * side, 18 + 6 * side) =
        model.interaction_contact_map[side];
    target.segment<6>(6 * side) -= model.interaction_bias[side];
  }
  wrench.block<12, 12>(0, 30) =
      -Eigen::Matrix<double, 12, 12>::Identity();
  wrench *= transform;
  Eigen::Matrix<double, 12, 1> wrench_scale;
  for (int side = 0; side < 2; ++side) {
    for (int index = 0; index < 6; ++index) {
      wrench_scale[6 * side + index] =
          wheel_leg::phase21_profile::kVariableScale[30 + index];
    }
  }
  addTask<12>(expected_h, expected_g, wrench, target, wrench_scale);
  Eigen::Matrix<double, 12, 42> slack =
      Eigen::Matrix<double, 12, 42>::Zero();
  slack.block<12, 12>(0, 30).setIdentity();
  slack *= transform;
  addTask<12>(expected_h, expected_g, slack,
              Eigen::Matrix<double, 12, 1>::Zero(), wrench_scale);
  assert((minimal.h - expected_h).cwiseAbs().maxCoeff() <= 2.0e-12);
  assert((minimal.g - expected_g).cwiseAbs().maxCoeff() <= 2.0e-12);

  auto irrelevant = reference;
  irrelevant.base_x_acceleration_m_s2 = 7.0;
  irrelevant.base_height_acceleration_m_s2 = -8.0;
  irrelevant.orientation_acceleration_rad_s2 << 1.0, 2.0, 3.0;
  irrelevant.leg_acceleration_rad_s2 << 4.0, 5.0, 6.0, 7.0;
  const auto invariant = assembler.assemble(
      model, irrelevant, wheel_leg::WeightedWbcProfile::kPhase27Minimal);
  assert((minimal.h - invariant.h).cwiseAbs().maxCoeff() == 0.0);
  assert((minimal.g - invariant.g).cwiseAbs().maxCoeff() == 0.0);

  wheel_leg::WeightedWbcController controller(
      wheel_leg::WeightedWbcProfile::kPhase27Minimal);
  const auto cold = controller.step(state, reference);
  assert(cold.ok());
  assert(cold.hard_violation <= 2.0e-7);
  for (int joint = 0; joint < 6; ++joint) {
    assert(std::abs(cold.torque_nm[static_cast<std::size_t>(joint)] -
                    cold.physical_solution[12 + joint]) <= 1.0e-12);
  }
  assert((cold.realized_interaction_wrench_flu -
          reference.interaction_wrench_flu -
          cold.signed_interaction_slack_flu -
          cold.interaction_wrench_residual_flu).cwiseAbs().maxCoeff() <= 1.0e-12);
  for (auto task : {wheel_leg::WeightedWbcController::Task::kBaseX,
                    wheel_leg::WeightedWbcController::Task::kHeight,
                    wheel_leg::WeightedWbcController::Task::kOrientation,
                    wheel_leg::WeightedWbcController::Task::kLeg}) {
    const auto index = static_cast<std::size_t>(task);
    assert(cold.task_max_abs_normalized_residual[index] == 0.0);
    assert(cold.task_normalized_squared_cost[index] == 0.0);
  }
  controller.reset();
  const auto repeated = controller.step(state, reference);
  assert(repeated.ok());
  assert((cold.physical_solution - repeated.physical_solution)
             .cwiseAbs().maxCoeff() <= 1.0e-12);

  for (const auto &dynamic_state : readDynamicStates()) {
    controller.reset();
    const auto result = controller.step(dynamic_state, reference);
    assert(result.ok());
    assert(result.hard_violation <= 2.0e-7);
    assert(result.physical_solution.allFinite());
  }

  std::vector<double> timing_ms;
  timing_ms.reserve(300);
  for (int iteration = 0; iteration < 300; ++iteration) {
    auto request = reference;
    request.interaction_wrench_flu[0] = 0.2 * std::sin(0.02 * iteration);
    request.interaction_wrench_flu[6] = request.interaction_wrench_flu[0];
    const auto start = std::chrono::steady_clock::now();
    const auto result = controller.step(state, request);
    const auto end = std::chrono::steady_clock::now();
    assert(result.ok());
    timing_ms.push_back(
        std::chrono::duration<double, std::milli>(end - start).count());
  }
  std::sort(timing_ms.begin(), timing_ms.end());
  assert(timing_ms.back() <= 10.0);
  std::cout << "phase27 minimal WBC: PASS p99_ms=" << timing_ms[296]
            << " max_ms=" << timing_ms.back()
            << " hard=" << cold.hard_violation
            << " wrench_residual="
            << cold.interaction_wrench_residual_flu.cwiseAbs().maxCoeff()
            << " slack=" << cold.maximum_normalized_slack << '\n';
  return 0;
}
