#include "wheel_leg_core/weighted_wbc_problem.hpp"

#include <algorithm>
#include <cmath>

#include "nominal_wbc_profile_data.hpp"

namespace wheel_leg {
namespace {

constexpr double kInfinity = 1.0e30;
constexpr double kContactScale = 10.0;
constexpr double kBaseLinearScale = 10.0;
constexpr double kBaseAngularScale = 20.0;
constexpr double kLegScale = 50.0;
constexpr double kRegularization = 1.0e-6;

using Matrix42 = WeightedWbcProblem::Matrix42;
using Vector42 = WeightedWbcProblem::Vector42;

template <int Rows>
void addTask(Matrix42 &h, Vector42 &g,
             const Eigen::Matrix<double, Rows, 42> &physical,
             const Eigen::Matrix<double, Rows, 1> &target,
             const Eigen::Matrix<double, Rows, 1> &scale) {
  Eigen::Matrix<double, Rows, 42> normalized;
  Eigen::Matrix<double, Rows, 1> normalized_target;
  for (int row = 0; row < Rows; ++row) {
    normalized.row(row) = physical.row(row) / scale[row];
    normalized_target[row] = target[row] / scale[row];
  }
  h.noalias() += normalized.transpose() * normalized;
  g.noalias() -= normalized.transpose() * normalized_target;
}

bool finite(const WeightedWbcProblem::Result &result) {
  return result.h.allFinite() && result.g.allFinite() && result.a.allFinite() &&
         result.lower.allFinite() && result.upper.allFinite();
}

}  // namespace

WeightedWbcProblem::Result WeightedWbcProblem::assemble(
    const NominalWbcModel::Result &model,
    const WbcReference &reference) const {
  Result result;
  result.model_status = model.status;
  if (!model.ok()) return result;
  if (!reference.orientation_acceleration_rad_s2.allFinite() ||
      !reference.leg_acceleration_rad_s2.allFinite() ||
      !reference.interaction_wrench_flu.allFinite() ||
      !std::isfinite(reference.base_x_acceleration_m_s2) ||
      !std::isfinite(reference.base_height_acceleration_m_s2)) {
    result.status = Status::kNonFinite;
    return result;
  }

  Eigen::Matrix<double, 42, 1> variable_scale;
  for (int index = 0; index < 42; ++index) {
    variable_scale[index] = phase21_profile::kVariableScale[index];
  }
  Eigen::Matrix<double, 12, 42> dynamics =
      Eigen::Matrix<double, 12, 42>::Zero();
  dynamics.leftCols<12>() = model.mass;
  dynamics.middleCols<6>(12) = -model.actuation;
  dynamics.middleCols<6>(18) = -model.wrench_map[0];
  dynamics.middleCols<6>(24) = -model.wrench_map[1];
  for (int row = 0; row < 12; ++row) {
    const double row_scale = phase21_profile::kDynamicsRowScale[row];
    result.a.row(row) =
        dynamics.row(row) * variable_scale.asDiagonal() / row_scale;
    result.lower[row] = result.upper[row] = -model.bias[row] / row_scale;
  }
  for (int joint = 0; joint < 6; ++joint) {
    const int row = 12 + joint;
    result.a(row, 12 + joint) =
        variable_scale[12 + joint] / phase21_profile::kTorqueLimit[joint];
    result.lower[row] = -1.0;
    result.upper[row] = 1.0;
  }
  for (int side = 0; side < 2; ++side) {
    const int variable_start = 18 + 6 * side;
    const int row_start = 18 + 37 * side;
    for (int cone_row = 0; cone_row < 37; ++cone_row) {
      double norm_squared = 0.0;
      for (int column = 0; column < 6; ++column) {
        const double value = phase21_profile::kWrenchCone[cone_row][column] *
                             variable_scale[variable_start + column];
        result.a(row_start + cone_row, variable_start + column) = value;
        norm_squared += value * value;
      }
      result.a.row(row_start + cone_row) /= std::sqrt(norm_squared);
      result.lower[row_start + cone_row] = -kInfinity;
      result.upper[row_start + cone_row] = 0.0;
    }
  }
  for (int coordinate = 0; coordinate < 12; ++coordinate) {
    const int row = 92 + coordinate;
    result.a(row, coordinate) = variable_scale[coordinate] /
                                phase21_profile::kAccelerationLimit[coordinate];
    result.lower[row] = -1.0;
    result.upper[row] = 1.0;
  }

  result.h.setIdentity();
  result.h *= kRegularization;
  result.g.setZero();
  const Eigen::DiagonalMatrix<double, 42> transform(variable_scale);

  Eigen::Matrix<double, 6, 42> contact =
      Eigen::Matrix<double, 6, 42>::Zero();
  contact.block<3, 12>(0, 0) = model.contact_jacobian[0];
  contact.block<3, 12>(3, 0) = model.contact_jacobian[1];
  contact *= transform;
  Eigen::Matrix<double, 6, 1> contact_target;
  contact_target << -model.contact_bias[0], -model.contact_bias[1];
  addTask<6>(result.h, result.g, contact, contact_target,
             Eigen::Matrix<double, 6, 1>::Constant(kContactScale));

  Eigen::Matrix<double, 1, 42> base_x =
      Eigen::Matrix<double, 1, 42>::Zero();
  base_x(0, 0) = variable_scale[0];
  addTask<1>(result.h, result.g, base_x,
             Eigen::Matrix<double, 1, 1>::Constant(
                 reference.base_x_acceleration_m_s2),
             Eigen::Matrix<double, 1, 1>::Constant(kBaseLinearScale));
  Eigen::Matrix<double, 1, 42> height =
      Eigen::Matrix<double, 1, 42>::Zero();
  height(0, 2) = variable_scale[2];
  addTask<1>(result.h, result.g, height,
             Eigen::Matrix<double, 1, 1>::Constant(
                 reference.base_height_acceleration_m_s2),
             Eigen::Matrix<double, 1, 1>::Constant(kBaseLinearScale));
  Eigen::Matrix<double, 3, 42> orientation =
      Eigen::Matrix<double, 3, 42>::Zero();
  for (int index = 0; index < 3; ++index) {
    orientation(index, 3 + index) = variable_scale[3 + index];
  }
  addTask<3>(result.h, result.g, orientation,
             reference.orientation_acceleration_rad_s2,
             Eigen::Vector3d::Constant(kBaseAngularScale));
  Eigen::Matrix<double, 4, 42> leg =
      Eigen::Matrix<double, 4, 42>::Zero();
  constexpr int kLegIndices[4]{0, 1, 3, 4};
  for (int index = 0; index < 4; ++index) {
    leg(index, 6 + kLegIndices[index]) =
        variable_scale[6 + kLegIndices[index]];
  }
  addTask<4>(result.h, result.g, leg, reference.leg_acceleration_rad_s2,
             Eigen::Vector4d::Constant(kLegScale));

  Eigen::Matrix<double, 12, 42> wrench =
      Eigen::Matrix<double, 12, 42>::Zero();
  for (int side = 0; side < 2; ++side) {
    wrench.block<6, 6>(6 * side, 18 + 6 * side) =
        model.wrench_flu_map[side];
  }
  wrench.block<12, 12>(0, 30) =
      -Eigen::Matrix<double, 12, 12>::Identity();
  wrench *= transform;
  Eigen::Matrix<double, 12, 1> wrench_scale;
  for (int side = 0; side < 2; ++side) {
    for (int index = 0; index < 6; ++index) {
      wrench_scale[6 * side + index] =
          phase21_profile::kVariableScale[30 + index];
    }
  }
  addTask<12>(result.h, result.g, wrench,
              reference.interaction_wrench_flu, wrench_scale);
  Eigen::Matrix<double, 12, 42> slack =
      Eigen::Matrix<double, 12, 42>::Zero();
  slack.block<12, 12>(0, 30) =
      Eigen::Matrix<double, 12, 12>::Identity();
  slack *= transform;
  addTask<12>(result.h, result.g, slack,
              Eigen::Matrix<double, 12, 1>::Zero(), wrench_scale);

  if (!finite(result)) {
    result = Result{};
    result.status = Status::kNonFinite;
    return result;
  }
  result.status = Status::kOk;
  return result;
}

}  // namespace wheel_leg
