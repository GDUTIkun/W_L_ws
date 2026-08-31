#include "wheel_leg_core/weighted_wbc_problem.hpp"

#include <algorithm>
#include <cmath>

#include "nominal_wbc_profile_data.hpp"

namespace wheel_leg {
namespace {

constexpr double kInfinity = 1.0e30;
constexpr double kContactScale = 10.0;
constexpr double kWheelVerticalScale = 1.0;
constexpr double kWheelLongitudinalScale = 1.0;
constexpr double kWheelJointAccelerationScale = 20.0;
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

bool usesMinimalInteractionWrench(WeightedWbcProfile profile) {
  return profile == WeightedWbcProfile::kPhase27Minimal ||
         profile == WeightedWbcProfile::kPhase33ZetaManifold ||
         profile == WeightedWbcProfile::kPhase34XiTracking ||
         profile == WeightedWbcProfile::kPhase43NativeWheelRate ||
         profile == WeightedWbcProfile::kPhase43XiAndNativeWheelRate ||
         profile == WeightedWbcProfile::kPhase45ContactConsistentRolling ||
         profile == WeightedWbcProfile::kPhase46HipCommonSafeRolling ||
         profile == WeightedWbcProfile::kPhase46HipCommonIncrementLimitedRolling ||
         profile == WeightedWbcProfile::kPhase46PointRealizableRolling ||
         profile == WeightedWbcProfile::kPhase46ConstraintConsistentLegClosureReaction ||
         profile == WeightedWbcProfile::kPhase46MujocoContactResponse;
}

bool usesPointRealizableContact(WeightedWbcProfile profile) {
  return profile == WeightedWbcProfile::kPhase46PointRealizableRolling ||
         profile == WeightedWbcProfile::kPhase46ConstraintConsistentLegClosureReaction ||
         profile == WeightedWbcProfile::kPhase46MujocoContactResponse;
}

}  // namespace

NominalWbcModel::Matrix1x12 hipCommonSafeRollingMap(
    const NominalWbcModel::Matrix1x12 &map) {
  auto projected = map;
  const double hip_common = 0.5 * (map(0, 6) + map(0, 9));
  projected(0, 6) -= hip_common;
  projected(0, 9) -= hip_common;
  return projected;
}

WeightedWbcProblem::Result WeightedWbcProblem::assemble(
    const NominalWbcModel::Result &model,
    const WbcReference &reference, WeightedWbcProfile profile) const {
  Result result;
  result.model_status = model.status;
  if (!model.ok()) return result;
  if (!reference.orientation_acceleration_rad_s2.allFinite() ||
      !reference.leg_acceleration_rad_s2.allFinite() ||
      !reference.wheel_vertical_acceleration_m_s2.allFinite() ||
      !reference.wheel_longitudinal_acceleration_m_s2.allFinite() ||
      !reference.wheel_joint_acceleration_rad_s2.allFinite() ||
      !reference.rolling_acceleration_bias_m_s2.allFinite() ||
      !reference.rolling_acceleration_m_s2.allFinite() ||
      !reference.rolling_velocity_m_s.allFinite() ||
      !reference.interaction_wrench_flu.allFinite() ||
      (reference.hip_common_increment_limit_active &&
       !std::isfinite(reference.nominal_hip_common_acceleration_rad_s2)) ||
      !std::isfinite(reference.base_x_acceleration_m_s2) ||
      !std::isfinite(reference.base_height_acceleration_m_s2)) {
    result.status = Status::kNonFinite;
    return result;
  }
  if (profile == WeightedWbcProfile::kPhase46MujocoContactResponse) {
    if (!reference.primitive_contact_active) return result;
    if (reference.primitive_contact_row_count <= 0 ||
        reference.primitive_contact_row_count > 12) return result;
    if (!reference.primitive_contact_nudot.allFinite() ||
        !reference.primitive_contact_wrench.allFinite() ||
        !reference.primitive_contact_rhs.allFinite()) {
      result.status = Status::kNonFinite;
      return result;
    }
  }

  Eigen::Matrix<double, 42, 1> variable_scale;
  for (int index = 0; index < 42; ++index) {
    variable_scale[index] = phase21_profile::kVariableScale[index];
  }
  std::array<NominalWbcModel::Matrix6, 2> wrench_projector{
      NominalWbcModel::Matrix6::Identity(),
      NominalWbcModel::Matrix6::Identity()};
  if (usesPointRealizableContact(profile)) {
    for (int side = 0; side < 2; ++side) {
      wrench_projector[side] =
          model.point_force_wrench_projector[side];
    }
  }
  Eigen::Matrix<double, 12, 42> dynamics =
      Eigen::Matrix<double, 12, 42>::Zero();
  dynamics.leftCols<12>() = model.mass;
  dynamics.middleCols<6>(12) = -model.actuation;
  dynamics.middleCols<6>(18) = -model.wrench_map[0] * wrench_projector[0];
  dynamics.middleCols<6>(24) = -model.wrench_map[1] * wrench_projector[1];
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
      Eigen::Matrix<double, 1, 6> physical;
      for (int column = 0; column < 6; ++column) {
        physical[column] = phase21_profile::kWrenchCone[cone_row][column];
      }
      physical *= wrench_projector[side];
      double norm_squared = 0.0;
      for (int column = 0; column < 6; ++column) {
        const double value = physical[column] *
                             variable_scale[variable_start + column];
        result.a(row_start + cone_row, variable_start + column) = value;
        norm_squared += value * value;
      }
      if (norm_squared > 0.0) {
        result.a.row(row_start + cone_row) /= std::sqrt(norm_squared);
      }
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
  result.lower[104] = -kInfinity;
  result.upper[104] = kInfinity;
  for (int row = 105; row < kConstraintCount; ++row) {
    result.lower[row] = -kInfinity;
    result.upper[row] = kInfinity;
  }
  if (profile == WeightedWbcProfile::kPhase46HipCommonIncrementLimitedRolling &&
      reference.hip_common_increment_limit_active) {
    result.a(104, 6) = 0.5 * variable_scale[6];
    result.a(104, 9) = 0.5 * variable_scale[9];
    result.lower[104] = result.upper[104] =
        reference.nominal_hip_common_acceleration_rad_s2;
  }
  if (profile == WeightedWbcProfile::kPhase46MujocoContactResponse) {
    for (int row = 0; row < reference.primitive_contact_row_count; ++row) {
      const int constraint_row = 105 + row;
      Eigen::Matrix<double, 1, 42> physical =
          Eigen::Matrix<double, 1, 42>::Zero();
      physical.block<1, 12>(0, 0) =
          reference.primitive_contact_nudot.row(row);
      physical.block<1, 12>(0, 18) =
          reference.primitive_contact_wrench.row(row);
      const auto scaled = physical * variable_scale.asDiagonal();
      const double norm = scaled.norm();
      if (!std::isfinite(norm) || norm == 0.0) {
        result.status = Status::kNonFinite;
        return result;
      }
      result.a.row(constraint_row) = scaled / norm;
      result.lower[constraint_row] = result.upper[constraint_row] =
          reference.primitive_contact_rhs[row] / norm;
    }
  }

  result.h.setIdentity();
  result.h *= kRegularization;
  if (usesMinimalInteractionWrench(profile)) {
    result.h.bottomRightCorner<12, 12>().setZero();
  }
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

  if (profile == WeightedWbcProfile::kPhase33ZetaManifold) {
    Eigen::Matrix<double, 2, 42> wheel_vertical =
        Eigen::Matrix<double, 2, 42>::Zero();
    Eigen::Vector2d wheel_vertical_target =
        reference.wheel_vertical_acceleration_m_s2;
    for (int side = 0; side < 2; ++side) {
      wheel_vertical.block<1, 12>(side, 0) =
          model.wheel_vertical_acceleration_map[side];
      wheel_vertical_target[side] -=
          model.wheel_vertical_acceleration_bias_m_s2[side];
    }
    wheel_vertical *= transform;
    addTask<2>(result.h, result.g, wheel_vertical, wheel_vertical_target,
               Eigen::Vector2d::Constant(kWheelVerticalScale));
  }

  if (profile == WeightedWbcProfile::kPhase34XiTracking ||
      profile == WeightedWbcProfile::kPhase43XiAndNativeWheelRate ||
      profile == WeightedWbcProfile::kPhase45ContactConsistentRolling ||
      profile == WeightedWbcProfile::kPhase46HipCommonSafeRolling ||
      profile == WeightedWbcProfile::kPhase46HipCommonIncrementLimitedRolling ||
      profile == WeightedWbcProfile::kPhase46PointRealizableRolling ||
      profile == WeightedWbcProfile::kPhase46ConstraintConsistentLegClosureReaction ||
      profile == WeightedWbcProfile::kPhase46MujocoContactResponse) {
    Eigen::Matrix<double, 2, 42> wheel_longitudinal =
        Eigen::Matrix<double, 2, 42>::Zero();
    Eigen::Vector2d wheel_longitudinal_target =
        reference.wheel_longitudinal_acceleration_m_s2;
    for (int side = 0; side < 2; ++side) {
      wheel_longitudinal.block<1, 12>(side, 0) =
          model.wheel_longitudinal_acceleration_map[side];
      wheel_longitudinal_target[side] -=
          model.wheel_longitudinal_acceleration_bias_m_s2[side];
    }
    wheel_longitudinal *= transform;
    addTask<2>(result.h, result.g, wheel_longitudinal,
               wheel_longitudinal_target,
               Eigen::Vector2d::Constant(kWheelLongitudinalScale));
  }

  if (profile == WeightedWbcProfile::kPhase45ContactConsistentRolling ||
      profile == WeightedWbcProfile::kPhase46HipCommonSafeRolling ||
      profile == WeightedWbcProfile::kPhase46HipCommonIncrementLimitedRolling ||
      profile == WeightedWbcProfile::kPhase46PointRealizableRolling ||
      profile == WeightedWbcProfile::kPhase46ConstraintConsistentLegClosureReaction ||
      profile == WeightedWbcProfile::kPhase46MujocoContactResponse) {
    Eigen::Matrix<double, 2, 42> rolling =
        Eigen::Matrix<double, 2, 42>::Zero();
    Eigen::Vector2d target = Eigen::Vector2d::Zero();
    for (int side = 0; side < 2; ++side) {
      if (!reference.rolling_acceleration_map[side].allFinite()) {
        result.status = Status::kNonFinite;
        return result;
      }
      if (!reference.rolling_task_active[side]) continue;
      rolling.block<1, 12>(side, 0) =
          profile == WeightedWbcProfile::kPhase46HipCommonSafeRolling
              ? hipCommonSafeRollingMap(reference.rolling_acceleration_map[side])
              : reference.rolling_acceleration_map[side];
      target[side] = reference.rolling_acceleration_m_s2[side] -
                     reference.rolling_acceleration_bias_m_s2[side];
    }
    rolling *= transform;
    addTask<2>(result.h, result.g, rolling, target,
               Eigen::Vector2d::Constant(kWheelLongitudinalScale));
  }

  if (profile == WeightedWbcProfile::kPhase43NativeWheelRate ||
      profile == WeightedWbcProfile::kPhase43XiAndNativeWheelRate) {
    Eigen::Matrix<double, 2, 42> wheel_joint =
        Eigen::Matrix<double, 2, 42>::Zero();
    wheel_joint(0, 8) = variable_scale[8];
    wheel_joint(1, 11) = variable_scale[11];
    addTask<2>(result.h, result.g, wheel_joint,
               reference.wheel_joint_acceleration_rad_s2,
               Eigen::Vector2d::Constant(kWheelJointAccelerationScale));
  }

  if (profile == WeightedWbcProfile::kNominal) {
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
  }

  Eigen::Matrix<double, 12, 42> wrench =
      Eigen::Matrix<double, 12, 42>::Zero();
  Eigen::Matrix<double, 12, 1> wrench_target =
      reference.interaction_wrench_flu;
  for (int side = 0; side < 2; ++side) {
    if (usesMinimalInteractionWrench(profile)) {
      wrench.block<6, 12>(6 * side, 0) =
          model.interaction_acceleration_map[side];
      wrench.block<6, 6>(6 * side, 18 + 6 * side) =
          model.interaction_contact_map[side] * wrench_projector[side];
      wrench_target.segment<6>(6 * side) -= model.interaction_bias[side];
    } else {
      wrench.block<6, 6>(6 * side, 18 + 6 * side) =
          model.wrench_flu_map[side] * wrench_projector[side];
    }
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
              wrench_target, wrench_scale);
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
