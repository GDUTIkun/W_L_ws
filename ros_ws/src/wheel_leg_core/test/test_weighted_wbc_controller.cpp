#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <fstream>
#include <string>

#include "wheel_leg_core/weighted_wbc_controller.hpp"

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

}  // namespace

int main() {
  std::ifstream input(WBC_PROBLEM_GOLDEN_PATH);
  std::string header;
  int count = 0;
  assert(input >> header >> count);
  assert(header == "WBC_PROBLEM_GOLDEN_V1" && count == 32);
  std::string case_id;
  assert(input >> case_id);
  auto state = readState(input);
  const auto initial_state = state;
  const auto reference = readReference(input);
  wheel_leg::WeightedWbcController controller;
  const auto cold = controller.step(state, reference);
  assert(cold.ok());
  assert(cold.hard_violation <= 2.0e-7);
  assert(cold.physical_solution.allFinite());
  assert(cold.primal_residual >= 0.0 && cold.dual_residual >= 0.0);
  for (std::size_t task = 0; task < wheel_leg::WeightedWbcController::kTaskCount; ++task) {
    assert(std::isfinite(cold.task_max_abs_normalized_residual[task]));
    assert(std::isfinite(cold.task_normalized_squared_cost[task]));
  }
  for (std::size_t joint = 0; joint < wheel_leg::kJointCount; ++joint) {
    assert(std::abs(cold.physical_solution[12 + static_cast<int>(joint)] -
                    cold.torque_nm[joint]) <= 1.0e-12);
  }
  const auto warm = controller.step(state, reference);
  assert(warm.ok());
  for (std::size_t joint = 0; joint < wheel_leg::kJointCount; ++joint) {
    assert(std::abs(cold.torque_nm[joint] - warm.torque_nm[joint]) <= 2.0e-5);
  }
  controller.reset();
  const auto repeated_cold = controller.step(state, reference);
  assert(repeated_cold.ok());
  for (std::size_t joint = 0; joint < wheel_leg::kJointCount; ++joint) {
    assert(std::abs(cold.torque_nm[joint] - repeated_cold.torque_nm[joint]) <=
           1.0e-12);
  }
  state.joint_position_rad[0] += 0.651;
  const auto rejected = controller.step(state, reference);
  assert(rejected.status ==
         wheel_leg::WeightedWbcController::Status::kModelRejected);
  for (const double torque : rejected.torque_nm) assert(torque == 0.0);

  wheel_leg::WeightedWbcController closure_controller(
      wheel_leg::WeightedWbcProfile::kPhase46ConstraintConsistentLegClosureReaction);
  const auto closure = closure_controller.step(initial_state, reference);
  assert(closure.ok());
  wheel_leg::NominalWbcModel model;
  const auto evaluated = model.evaluate(initial_state);
  assert(evaluated.ok());
  auto contact_reference = reference;
  contact_reference.primitive_contact_active = true;
  contact_reference.primitive_contact_row_count = 12;
  contact_reference.primitive_contact_wrench.setIdentity();
  contact_reference.primitive_contact_rhs =
      closure.physical_solution.segment<12>(18);
  wheel_leg::WeightedWbcController contact_controller(
      wheel_leg::WeightedWbcProfile::kPhase46MujocoContactResponse);
  const auto contact = contact_controller.step(initial_state, contact_reference);
  assert(contact.ok());
  assert(contact.contact_response_hard_residual.cwiseAbs().maxCoeff() <=
         1.0e-6);
  contact_reference.primitive_contact_active = false;
  const auto inactive = contact_controller.step(initial_state, contact_reference);
  assert(inactive.status ==
         wheel_leg::WeightedWbcController::Status::kProblemRejected);
  return 0;
}
