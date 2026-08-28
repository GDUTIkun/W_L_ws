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
    wheel_leg::WeightedWbcProblem::Matrix104x42 expected_a;
    wheel_leg::WeightedWbcProblem::Vector104 expected_lower;
    wheel_leg::WeightedWbcProblem::Vector104 expected_upper;
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
    assert((actual.a - expected_a).cwiseAbs().maxCoeff() <= 2.0e-9);
    assert((actual.lower - expected_lower).cwiseAbs().maxCoeff() <= 2.0e-9);
    assert((actual.upper - expected_upper).cwiseAbs().maxCoeff() <= 2.0e-9);
    if (index == 0) {
      equilibrium = state;
      equilibrium_reference = reference;
    }
  }

  equilibrium.joint_position_rad[0] += 0.651;
  const auto outside = model.evaluate(equilibrium);
  assert(outside.status ==
         wheel_leg::NominalWbcModel::Status::kOutsideWorkspace);
  assert(assembler.assemble(outside, equilibrium_reference).status ==
         wheel_leg::WeightedWbcProblem::Status::kModelRejected);
  equilibrium_reference.interaction_wrench_flu[0] =
      std::numeric_limits<double>::quiet_NaN();
  equilibrium.joint_position_rad[0] -= 0.651;
  assert(assembler.assemble(model.evaluate(equilibrium), equilibrium_reference).status ==
         wheel_leg::WeightedWbcProblem::Status::kNonFinite);
  return 0;
}
