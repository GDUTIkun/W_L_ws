#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <fstream>
#include <limits>
#include <string>

#include "wheel_leg_core/weighted_wbc_problem.hpp"

namespace {

template <typename Matrix>
void readMatrix(std::istream &input, Matrix &matrix) {
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      assert(input >> matrix(row, column));
    }
  }
}

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
  readMatrix(input, reference.orientation_acceleration_rad_s2);
  readMatrix(input, reference.leg_acceleration_rad_s2);
  readMatrix(input, reference.interaction_wrench_flu);
  return reference;
}

}  // namespace

int main() {
  std::ifstream input(WBC_PROBLEM_GOLDEN_PATH);
  assert(input);
  std::string header;
  int count = 0;
  assert(input >> header >> count);
  assert(header == "WBC_PROBLEM_GOLDEN_V1");
  assert(count == 32);
  wheel_leg::NominalWbcModel model;
  wheel_leg::WeightedWbcProblem assembler;
  wheel_leg::RobotState equilibrium;
  wheel_leg::WbcReference equilibrium_reference;
  for (int index = 0; index < count; ++index) {
    std::string case_id;
    assert(input >> case_id);
    auto state = readState(input);
    auto reference = readReference(input);
    wheel_leg::WeightedWbcProblem::Matrix42 expected_h;
    wheel_leg::WeightedWbcProblem::Vector42 expected_g;
    Eigen::Matrix<double, 104, 42> expected_a;
    Eigen::Matrix<double, 104, 1> expected_lower;
    Eigen::Matrix<double, 104, 1> expected_upper;
    readMatrix(input, expected_h);
    readMatrix(input, expected_g);
    readMatrix(input, expected_a);
    readMatrix(input, expected_lower);
    readMatrix(input, expected_upper);
    const auto evaluated = model.evaluate(state);
    assert(evaluated.ok());
    const auto actual = assembler.assemble(evaluated, reference);
    assert(actual.ok());
    assert((actual.h - expected_h).cwiseAbs().maxCoeff() <= 2.0e-10);
    assert((actual.g - expected_g).cwiseAbs().maxCoeff() <= 2.0e-8);
    assert((actual.a.topRows<104>() - expected_a).cwiseAbs().maxCoeff() <= 2.0e-9);
    assert((actual.lower.head<104>() - expected_lower).cwiseAbs().maxCoeff() <= 2.0e-9);
    assert((actual.upper.head<104>() - expected_upper).cwiseAbs().maxCoeff() <= 2.0e-9);
    if (index == 0) {
      equilibrium = state;
      equilibrium_reference = reference;
    }
  }

  equilibrium.joint_position_rad[0] += 0.651;
  const auto inspection = wheel_leg::NominalWbcModel::inspectWorkspace(equilibrium);
  assert(!inspection.inside());
  assert(inspection.first_failed_index == 0);
  assert(inspection.minimum_margin_index == 0);
  assert(std::abs(inspection.joint[0].signed_margin_rad + 0.001) <= 1.0e-12);
  const auto outside = model.evaluate(equilibrium);
  assert(outside.status ==
         wheel_leg::NominalWbcModel::Status::kOutsideWorkspace);
  assert(assembler.assemble(outside, equilibrium_reference).status ==
         wheel_leg::WeightedWbcProblem::Status::kModelRejected);
  equilibrium.joint_position_rad[0] -= 0.651;
  auto wheel_outside = equilibrium;
  wheel_outside.joint_position_rad[2] += 2.0 * std::acos(-1.0);
  assert(wheel_leg::NominalWbcModel::inspectWorkspace(wheel_outside).inside());
  assert(model.evaluate(wheel_outside).ok());
  wheel_outside.joint_position_rad[5] += 10.0 * std::acos(-1.0);
  assert(wheel_leg::NominalWbcModel::inspectWorkspace(wheel_outside).inside());
  assert(model.evaluate(wheel_outside).ok());
  wheel_outside.joint_position_rad[2] =
      std::numeric_limits<double>::infinity();
  assert(model.evaluate(wheel_outside).status ==
         wheel_leg::NominalWbcModel::Status::kInvalidState);
  equilibrium_reference.interaction_wrench_flu[0] =
      std::numeric_limits<double>::quiet_NaN();
  constexpr std::array<int, 4> kWorkspaceJoints{0, 1, 3, 4};
  for (int joint : kWorkspaceJoints) {
    const auto baseline = wheel_leg::NominalWbcModel::inspectWorkspace(equilibrium);
    const double lower = baseline.joint[joint].lower_bound_rad;
    const double upper = baseline.joint[joint].upper_bound_rad;
    equilibrium.joint_position_rad[joint] =
        baseline.joint[joint].equilibrium_rad + lower;
    assert(wheel_leg::NominalWbcModel::inspectWorkspace(equilibrium).inside());
    equilibrium.joint_position_rad[joint] -= 1.0e-9;
    auto failed = wheel_leg::NominalWbcModel::inspectWorkspace(equilibrium);
    assert(failed.first_failed_index == joint);
    equilibrium.joint_position_rad[joint] =
        baseline.joint[joint].equilibrium_rad + upper;
    assert(wheel_leg::NominalWbcModel::inspectWorkspace(equilibrium).inside());
    equilibrium.joint_position_rad[joint] += 1.0e-9;
    failed = wheel_leg::NominalWbcModel::inspectWorkspace(equilibrium);
    assert(failed.first_failed_index == joint);
    equilibrium.joint_position_rad[joint] = baseline.joint[joint].position_rad;
  }
  assert(assembler.assemble(model.evaluate(equilibrium), equilibrium_reference).status ==
         wheel_leg::WeightedWbcProblem::Status::kNonFinite);
  return 0;
}
