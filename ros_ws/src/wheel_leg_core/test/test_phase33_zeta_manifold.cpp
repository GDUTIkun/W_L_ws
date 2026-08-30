#ifdef NDEBUG
#undef NDEBUG
#endif

#include <Eigen/Geometry>
#include <Eigen/QR>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>

#include "nominal_wbc_profile_data.hpp"
#include "wheel_leg_core/weighted_wbc_controller.hpp"

namespace {

wheel_leg::RobotState readEquilibriumState() {
  std::ifstream input(WBC_PROBLEM_GOLDEN_PATH);
  std::string header;
  std::string case_id;
  int count = 0;
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

wheel_leg::WbcReference equilibriumReference() {
  wheel_leg::WbcReference reference;
  reference.interaction_wrench_flu <<
      0.0, 0.0, 27.675229491866027, 0.11327183296816838, 0.0, 0.0,
      0.0, 0.0, 28.714612508133982, 0.11327183296816838, 0.0, 0.0;
  return reference;
}

wheel_leg::RobotState displacedState(
    const wheel_leg::RobotState &initial,
    const wheel_leg::NominalWbcModel::Vector12 &acceleration, double time_s) {
  auto state = initial;
  const double half_time_squared = 0.5 * time_s * time_s;
  for (int index = 0; index < 3; ++index) {
    state.base_position_n_m[index] +=
        initial.base_linear_velocity_n_m_s[index] * time_s +
        acceleration[index] * half_time_squared;
    state.base_linear_velocity_n_m_s[index] += acceleration[index] * time_s;
    state.base_angular_velocity_n_rad_s[index] +=
        acceleration[3 + index] * time_s;
  }
  const Eigen::Quaterniond initial_rotation(
      initial.q_n_from_b[0], initial.q_n_from_b[1], initial.q_n_from_b[2],
      initial.q_n_from_b[3]);
  Eigen::Vector3d rotation_vector;
  for (int index = 0; index < 3; ++index) {
    rotation_vector[index] =
        initial.base_angular_velocity_n_rad_s[index] * time_s +
        acceleration[3 + index] * half_time_squared;
  }
  Eigen::Quaterniond increment = Eigen::Quaterniond::Identity();
  if (rotation_vector.norm() > 0.0) {
    increment = Eigen::Quaterniond(Eigen::AngleAxisd(
        rotation_vector.norm(), rotation_vector.normalized()));
  }
  const Eigen::Quaterniond rotation = (increment * initial_rotation).normalized();
  state.q_n_from_b = {rotation.w(), rotation.x(), rotation.y(), rotation.z()};
  for (int index = 0; index < 6; ++index) {
    state.joint_position_rad[index] +=
        initial.joint_velocity_rad_s[index] * time_s +
        acceleration[6 + index] * half_time_squared;
    state.joint_velocity_rad_s[index] += acceleration[6 + index] * time_s;
  }
  return state;
}

}  // namespace

int main() {
  auto state = readEquilibriumState();
  state.joint_velocity_rad_s = {0.08, -0.05, 0.0, -0.06, 0.04, 0.0};
  wheel_leg::NominalWbcModel model;
  const auto evaluated = model.evaluate(state);
  assert(evaluated.ok());
  Eigen::Matrix<double, 6, 12> contact_rows;
  contact_rows << evaluated.contact_jacobian[0], evaluated.contact_jacobian[1];
  double minimum_contact_span_residual = 1.0;
  for (int side = 0; side < 2; ++side) {
    const auto zeta_row = evaluated.wheel_vertical_acceleration_map[side];
    const Eigen::VectorXd coefficients =
        contact_rows.transpose().completeOrthogonalDecomposition().solve(
            zeta_row.transpose());
    const double relative_residual =
        (contact_rows.transpose() * coefficients - zeta_row.transpose()).norm() /
        zeta_row.norm();
    minimum_contact_span_residual =
        std::min(minimum_contact_span_residual, relative_residual);
  }
  assert(minimum_contact_span_residual >= 1.0e-3);

  wheel_leg::NominalWbcModel::Vector12 acceleration;
  acceleration << 0.3, -0.2, 0.4, 0.1, -0.15, 0.08,
      0.5, -0.7, 0.0, -0.4, 0.6, 0.0;
  constexpr double kStep = 1.0e-5;
  const auto plus = model.evaluate(displacedState(state, acceleration, kStep));
  const auto minus = model.evaluate(displacedState(state, acceleration, -kStep));
  assert(plus.ok() && minus.ok());
  double maximum_affine_error = 0.0;
  for (int side = 0; side < 2; ++side) {
    const double finite_difference =
        (plus.wheel_velocity_b_z_m_s[side] -
         minus.wheel_velocity_b_z_m_s[side]) /
        (2.0 * kStep);
    const double affine =
        (evaluated.wheel_vertical_acceleration_map[side] * acceleration)(0) +
        evaluated.wheel_vertical_acceleration_bias_m_s2[side];
    maximum_affine_error =
        std::max(maximum_affine_error, std::abs(finite_difference - affine));
  }
  assert(maximum_affine_error <= 2.0e-6);

  const auto reference = equilibriumReference();
  wheel_leg::WeightedWbcProblem assembler;
  const auto minimal = assembler.assemble(
      evaluated, reference, wheel_leg::WeightedWbcProfile::kPhase27Minimal);
  const auto manifold = assembler.assemble(
      evaluated, reference, wheel_leg::WeightedWbcProfile::kPhase33ZetaManifold);
  assert(minimal.ok() && manifold.ok());
  assert((minimal.a - manifold.a).cwiseAbs().maxCoeff() == 0.0);
  assert((minimal.lower - manifold.lower).cwiseAbs().maxCoeff() == 0.0);
  assert((minimal.upper - manifold.upper).cwiseAbs().maxCoeff() == 0.0);

  Eigen::Matrix<double, 2, 42> task = Eigen::Matrix<double, 2, 42>::Zero();
  Eigen::Vector2d target;
  for (int side = 0; side < 2; ++side) {
    task.block<1, 12>(side, 0) =
        evaluated.wheel_vertical_acceleration_map[side];
    target[side] = -evaluated.wheel_vertical_acceleration_bias_m_s2[side];
  }
  for (int column = 0; column < 42; ++column) {
    task.col(column) *= wheel_leg::phase21_profile::kVariableScale[column];
  }
  const auto expected_h_delta = task.transpose() * task;
  const auto expected_g_delta = -task.transpose() * target;
  assert((manifold.h - minimal.h - expected_h_delta)
             .cwiseAbs().maxCoeff() <= 2.0e-12);
  assert((manifold.g - minimal.g - expected_g_delta)
             .cwiseAbs().maxCoeff() <= 2.0e-12);
  assert((task.block<2, 12>(0, 0).norm() > 0.0));

  auto irrelevant = reference;
  irrelevant.base_x_acceleration_m_s2 = 5.0;
  irrelevant.base_height_acceleration_m_s2 = -6.0;
  irrelevant.orientation_acceleration_rad_s2 << 1.0, 2.0, 3.0;
  irrelevant.leg_acceleration_rad_s2 << 4.0, 5.0, 6.0, 7.0;
  const auto invariant = assembler.assemble(
      evaluated, irrelevant,
      wheel_leg::WeightedWbcProfile::kPhase33ZetaManifold);
  assert((manifold.h - invariant.h).cwiseAbs().maxCoeff() == 0.0);
  assert((manifold.g - invariant.g).cwiseAbs().maxCoeff() == 0.0);
  auto nonfinite = reference;
  nonfinite.wheel_vertical_acceleration_m_s2[0] =
      std::numeric_limits<double>::quiet_NaN();
  assert(assembler.assemble(
      evaluated, nonfinite,
      wheel_leg::WeightedWbcProfile::kPhase33ZetaManifold).status ==
      wheel_leg::WeightedWbcProblem::Status::kNonFinite);

  wheel_leg::WeightedWbcController controller(
      wheel_leg::WeightedWbcProfile::kPhase33ZetaManifold);
  const auto baseline = controller.step(state, reference);
  assert(baseline.ok());
  assert(baseline.hard_violation <= 2.0e-7);
  controller.reset();
  auto request = reference;
  request.wheel_vertical_acceleration_m_s2[0] = 0.1;
  const auto positive = controller.step(state, request);
  controller.reset();
  request.wheel_vertical_acceleration_m_s2[0] = -0.1;
  const auto negative = controller.step(state, request);
  assert(positive.ok() && negative.ok());
  const Eigen::Vector2d response =
      (positive.wheel_vertical_acceleration_m_s2 -
       negative.wheel_vertical_acceleration_m_s2) /
      0.2;
  const double self_gain = response[0];
  const double cross_ratio = std::abs(response[1]) / std::abs(self_gain);
  const double wrench_change =
      (positive.realized_interaction_wrench_flu -
       negative.realized_interaction_wrench_flu).norm() /
      std::max(1.0, baseline.realized_interaction_wrench_flu.norm());
  assert(self_gain > 0.0);
  assert(positive.hard_violation <= 2.0e-7 &&
         negative.hard_violation <= 2.0e-7);

  std::cout << std::setprecision(17)
            << "phase33 zeta manifold: PASS affine_error="
            << maximum_affine_error << " zeta_ref_left="
            << evaluated.wheel_position_b_z_m[0] << " zeta_ref_right="
            << evaluated.wheel_position_b_z_m[1] << " authority_self="
            << self_gain << " authority_cross_ratio=" << cross_ratio
            << " wrench_change=" << wrench_change
            << " contact_span_residual=" << minimum_contact_span_residual
            << '\n';
  return 0;
}
