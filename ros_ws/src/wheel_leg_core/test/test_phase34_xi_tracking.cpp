#ifdef NDEBUG
#undef NDEBUG
#endif

#include <Eigen/Geometry>

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
    state.base_angular_velocity_n_rad_s[index] += acceleration[3 + index] * time_s;
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
        (plus.wheel_velocity_b_x_m_s[side] -
         minus.wheel_velocity_b_x_m_s[side]) /
        (2.0 * kStep);
    const double affine =
        (evaluated.wheel_longitudinal_acceleration_map[side] * acceleration)(0) +
        evaluated.wheel_longitudinal_acceleration_bias_m_s2[side];
    maximum_affine_error =
        std::max(maximum_affine_error, std::abs(finite_difference - affine));
  }
  assert(maximum_affine_error <= 2.0e-6);

  const auto reference = equilibriumReference();
  wheel_leg::WeightedWbcProblem assembler;
  const auto minimal = assembler.assemble(
      evaluated, reference, wheel_leg::WeightedWbcProfile::kPhase27Minimal);
  const auto tracking = assembler.assemble(
      evaluated, reference, wheel_leg::WeightedWbcProfile::kPhase34XiTracking);
  assert(minimal.ok() && tracking.ok());
  assert((minimal.a - tracking.a).cwiseAbs().maxCoeff() == 0.0);
  assert((minimal.lower - tracking.lower).cwiseAbs().maxCoeff() == 0.0);
  assert((minimal.upper - tracking.upper).cwiseAbs().maxCoeff() == 0.0);

  Eigen::Matrix<double, 2, 42> task = Eigen::Matrix<double, 2, 42>::Zero();
  Eigen::Vector2d target;
  for (int side = 0; side < 2; ++side) {
    task.block<1, 12>(side, 0) =
        evaluated.wheel_longitudinal_acceleration_map[side];
    target[side] = -evaluated.wheel_longitudinal_acceleration_bias_m_s2[side];
  }
  for (int column = 0; column < 42; ++column) {
    task.col(column) *= wheel_leg::phase21_profile::kVariableScale[column];
  }
  assert((tracking.h - minimal.h - task.transpose() * task)
             .cwiseAbs().maxCoeff() <= 2.0e-12);
  assert((tracking.g - minimal.g + task.transpose() * target)
             .cwiseAbs().maxCoeff() <= 2.0e-12);

  auto rate_reference = reference;
  rate_reference.wheel_joint_acceleration_rad_s2 << -0.7, 0.4;
  const auto rate = assembler.assemble(
      evaluated, rate_reference,
      wheel_leg::WeightedWbcProfile::kPhase43NativeWheelRate);
  const auto combined = assembler.assemble(
      evaluated, rate_reference,
      wheel_leg::WeightedWbcProfile::kPhase43XiAndNativeWheelRate);
  assert(rate.ok() && combined.ok());
  Eigen::Matrix<double, 2, 42> rate_task =
      Eigen::Matrix<double, 2, 42>::Zero();
  rate_task(0, 8) = wheel_leg::phase21_profile::kVariableScale[8] / 20.0;
  rate_task(1, 11) = wheel_leg::phase21_profile::kVariableScale[11] / 20.0;
  const Eigen::Vector2d normalized_rate_target =
      rate_reference.wheel_joint_acceleration_rad_s2 / 20.0;
  assert((rate.h - minimal.h - rate_task.transpose() * rate_task)
             .cwiseAbs().maxCoeff() <= 2.0e-12);
  assert((rate.g - minimal.g +
          rate_task.transpose() * normalized_rate_target)
             .cwiseAbs().maxCoeff() <= 2.0e-12);
  assert((combined.h - tracking.h - rate.h + minimal.h)
             .cwiseAbs().maxCoeff() <= 2.0e-12);
  assert((combined.g - tracking.g - rate.g + minimal.g)
             .cwiseAbs().maxCoeff() <= 2.0e-12);

  auto nonfinite = reference;
  nonfinite.wheel_longitudinal_acceleration_m_s2[0] =
      std::numeric_limits<double>::quiet_NaN();
  assert(assembler.assemble(
      evaluated, nonfinite,
      wheel_leg::WeightedWbcProfile::kPhase34XiTracking).status ==
      wheel_leg::WeightedWbcProblem::Status::kNonFinite);
  nonfinite = reference;
  nonfinite.wheel_joint_acceleration_rad_s2[1] =
      std::numeric_limits<double>::infinity();
  assert(assembler.assemble(
      evaluated, nonfinite,
      wheel_leg::WeightedWbcProfile::kPhase43NativeWheelRate).status ==
      wheel_leg::WeightedWbcProblem::Status::kNonFinite);

  wheel_leg::WeightedWbcController controller(
      wheel_leg::WeightedWbcProfile::kPhase34XiTracking);
  const auto baseline = controller.step(state, reference);
  assert(baseline.ok());
  Eigen::Matrix2d response;
  const std::array<Eigen::Vector2d, 2> directions{
      Eigen::Vector2d(1.0, 1.0), Eigen::Vector2d(-1.0, 1.0)};
  for (int channel = 0; channel < 2; ++channel) {
    controller.reset();
    auto positive_reference = reference;
    positive_reference.wheel_longitudinal_acceleration_m_s2 =
        0.1 * directions[channel];
    const auto positive = controller.step(state, positive_reference);
    controller.reset();
    auto negative_reference = reference;
    negative_reference.wheel_longitudinal_acceleration_m_s2 =
        -0.1 * directions[channel];
    const auto negative = controller.step(state, negative_reference);
    assert(positive.ok() && negative.ok());
    const Eigen::Vector2d per_side =
        (positive.wheel_longitudinal_acceleration_m_s2 -
         negative.wheel_longitudinal_acceleration_m_s2) /
        0.2;
    response(0, channel) = 0.5 * (per_side[0] + per_side[1]);
    response(1, channel) = 0.5 * (per_side[1] - per_side[0]);
  }
  assert(response(0, 0) > 0.0 && response(1, 1) > 0.0);
  std::cout << std::setprecision(17)
            << "phase34 xi tracking component: PASS affine_error="
            << maximum_affine_error << " response=" << response << '\n';
  return 0;
}
