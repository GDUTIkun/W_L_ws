#include "wheel_leg_core/nominal_wbc_model.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>

#include "nominal_wbc_profile_data.hpp"

namespace wheel_leg {
namespace {

using Matrix3x16 = Eigen::Matrix<double, 3, 16>;
using Vector16 = Eigen::Matrix<double, 16, 1>;
using Matrix16 = Eigen::Matrix<double, 16, 16>;

// Solve materially below the 1e-10 acceptance gate so downstream task
// gradients do not amplify a merely gate-level closure residual.
constexpr double kClosureTolerance = 1.0e-12;
constexpr double kMinimumPassiveSingularValue = 0.005;
constexpr double kMaximumPassiveCondition = 40.0;
// Phase46 compatible-H0/tick0 actual two-point contact-line offsets from the
// production wrench reference, expressed in each contact frame. These are
// frozen evidence inputs, not a general contact estimator.
constexpr std::array<double, 2> kPhase46ActualContactLineNormalOffsetM{
    0.00021526549679640877, 0.0001520424481146268};
constexpr int kMaximumReconstructionIterations = 30;

struct BodyState {
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d linear_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_velocity{Eigen::Vector3d::Zero()};
  Eigen::Vector3d linear_acceleration{Eigen::Vector3d::Zero()};
  Eigen::Vector3d angular_acceleration{Eigen::Vector3d::Zero()};
  Matrix3x16 linear_jacobian{Matrix3x16::Zero()};
  Matrix3x16 angular_jacobian{Matrix3x16::Zero()};
};

using Kinematics = std::array<BodyState, 12>;

Eigen::Matrix3d skew(const Eigen::Vector3d &value) {
  Eigen::Matrix3d result;
  result << 0.0, -value.z(), value.y(), value.z(), 0.0, -value.x(),
      -value.y(), value.x(), 0.0;
  return result;
}

Eigen::Matrix3d quaternion(const std::array<double, 4> &value) {
  Eigen::Quaterniond q(value[0], value[1], value[2], value[3]);
  return q.normalized().toRotationMatrix();
}

Eigen::Matrix3d quaternion(const double *value) {
  Eigen::Quaterniond q(value[0], value[1], value[2], value[3]);
  return q.normalized().toRotationMatrix();
}

Eigen::Vector3d vector3(const double *value) {
  return Eigen::Vector3d(value[0], value[1], value[2]);
}

Eigen::Vector3d pointPosition(
    const BodyState &body, const Eigen::Vector3d &local) {
  return body.position + body.rotation * local;
}

Matrix3x16 pointJacobian(
    const BodyState &body, const Eigen::Vector3d &local) {
  const Eigen::Vector3d offset = body.rotation * local;
  return body.linear_jacobian - skew(offset) * body.angular_jacobian;
}

Eigen::Vector3d pointAcceleration(
    const BodyState &body, const Eigen::Vector3d &local) {
  const Eigen::Vector3d offset = body.rotation * local;
  return body.linear_acceleration + body.angular_acceleration.cross(offset) +
         body.angular_velocity.cross(body.angular_velocity.cross(offset));
}

Kinematics forwardKinematics(
    const Eigen::Vector3d &base_control_position,
    const Eigen::Matrix3d &base_rotation,
    const Eigen::Matrix<double, 10, 1> &joint_position,
    const Vector16 &velocity,
    const Vector16 &acceleration = Vector16::Zero()) {
  Kinematics bodies;
  const Eigen::Vector3d control_local(
      phase21_profile::kBaseControlPosition[0],
      phase21_profile::kBaseControlPosition[1],
      phase21_profile::kBaseControlPosition[2]);
  const Eigen::Vector3d control_offset = base_rotation * control_local;
  BodyState &base = bodies[1];
  base.rotation = base_rotation;
  base.position = base_control_position - control_offset;
  base.linear_velocity = velocity.head<3>() +
                         control_offset.cross(velocity.segment<3>(3));
  base.angular_velocity = velocity.segment<3>(3);
  base.angular_acceleration = acceleration.segment<3>(3);
  base.linear_acceleration = acceleration.head<3>() +
      control_offset.cross(base.angular_acceleration) -
      base.angular_velocity.cross(base.angular_velocity.cross(control_offset));
  base.linear_jacobian.block<3, 3>(0, 0).setIdentity();
  base.linear_jacobian.block<3, 3>(0, 3) = skew(control_offset);
  base.angular_jacobian.block<3, 3>(0, 3).setIdentity();

  for (int body_id = 2; body_id <= 11; ++body_id) {
    const int body_index = body_id - 1;
    const auto &raw = phase21_profile::kBody[body_index];
    const int parent_id = phase21_profile::kBodyParent[body_index];
    const BodyState &parent = bodies[parent_id];
    BodyState &body = bodies[body_id];
    const Eigen::Vector3d fixed_position = vector3(raw.data());
    const Eigen::Matrix3d fixed_rotation = quaternion(raw.data() + 3);
    const int joint = body_id - 2;
    const Eigen::Vector3d offset = parent.rotation * fixed_position;
    const Eigen::Vector3d axis =
        parent.rotation * fixed_rotation * Eigen::Vector3d::UnitZ();
    body.position = parent.position + offset;
    body.rotation = parent.rotation * fixed_rotation *
        Eigen::AngleAxisd(joint_position[joint], Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    body.angular_velocity =
        parent.angular_velocity + axis * velocity[6 + joint];
    body.linear_velocity =
        parent.linear_velocity + parent.angular_velocity.cross(offset);
    body.angular_acceleration = parent.angular_acceleration +
        parent.angular_velocity.cross(axis * velocity[6 + joint]) +
        axis * acceleration[6 + joint];
    body.linear_acceleration = parent.linear_acceleration +
        parent.angular_acceleration.cross(offset) +
        parent.angular_velocity.cross(parent.angular_velocity.cross(offset));
    body.linear_jacobian =
        parent.linear_jacobian - skew(offset) * parent.angular_jacobian;
    body.angular_jacobian = parent.angular_jacobian;
    body.angular_jacobian.col(6 + joint) = axis;
  }
  return bodies;
}

Eigen::Matrix<double, 6, 1> closureResidual(const Kinematics &bodies) {
  Eigen::Matrix<double, 6, 1> residual;
  for (int side = 0; side < 2; ++side) {
    const int first = 2 * side;
    const int second = first + 1;
    const auto &first_local = phase21_profile::kClosurePosition[first];
    const auto &second_local = phase21_profile::kClosurePosition[second];
    residual.segment<3>(3 * side) =
        pointPosition(bodies[phase21_profile::kClosureBody[first]],
                      Eigen::Vector3d(first_local[0], first_local[1], first_local[2])) -
        pointPosition(bodies[phase21_profile::kClosureBody[second]],
                      Eigen::Vector3d(second_local[0], second_local[1], second_local[2]));
  }
  return residual;
}

Eigen::Matrix<double, 6, 16> closureJacobian(const Kinematics &bodies) {
  Eigen::Matrix<double, 6, 16> result;
  for (int side = 0; side < 2; ++side) {
    const int first = 2 * side;
    const int second = first + 1;
    const auto &first_local = phase21_profile::kClosurePosition[first];
    const auto &second_local = phase21_profile::kClosurePosition[second];
    result.block<3, 16>(3 * side, 0) =
        pointJacobian(bodies[phase21_profile::kClosureBody[first]],
                      Eigen::Vector3d(first_local[0], first_local[1], first_local[2])) -
        pointJacobian(bodies[phase21_profile::kClosureBody[second]],
                      Eigen::Vector3d(second_local[0], second_local[1], second_local[2]));
  }
  return result;
}

Eigen::Matrix<double, 6, 1> closureBias(const Kinematics &bodies) {
  Eigen::Matrix<double, 6, 1> result;
  for (int side = 0; side < 2; ++side) {
    const int first = 2 * side;
    const int second = first + 1;
    const auto &first_local = phase21_profile::kClosurePosition[first];
    const auto &second_local = phase21_profile::kClosurePosition[second];
    result.segment<3>(3 * side) =
        pointAcceleration(bodies[phase21_profile::kClosureBody[first]],
                          Eigen::Vector3d(first_local[0], first_local[1], first_local[2])) -
        pointAcceleration(bodies[phase21_profile::kClosureBody[second]],
                          Eigen::Vector3d(second_local[0], second_local[1], second_local[2]));
  }
  return result;
}

struct ContactGeometry {
  Eigen::Vector3d center;
  Eigen::Vector3d axis;
  Eigen::Vector3d radial;
  Eigen::Matrix3d frame;
  double projection{0.0};
  double mid{0.0};
  int body_id{0};
  Eigen::Vector3d geom_local;
};

ContactGeometry contactGeometry(const Kinematics &bodies, int side) {
  ContactGeometry result;
  result.body_id = phase21_profile::kWheelBody[side];
  const auto &raw = phase21_profile::kBody[result.body_id - 1];
  result.geom_local = vector3(raw.data() + 8);
  const Eigen::Matrix3d geom_rotation =
      bodies[result.body_id].rotation * quaternion(raw.data() + 11);
  const Eigen::Vector3d geom_center =
      pointPosition(bodies[result.body_id], result.geom_local);
  result.axis = geom_rotation.col(0);
  const Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  const double dot = result.axis.dot(normal);
  result.projection = std::sqrt(std::max(0.0, 1.0 - dot * dot));
  if (result.projection <= 1.0e-6) return result;
  const Eigen::Vector3d rolling = result.axis.cross(normal) / result.projection;
  const Eigen::Vector3d lateral = normal.cross(rolling);
  result.radial = (normal - dot * result.axis) / result.projection;
  const auto &bounds = phase21_profile::kWheelAxisBounds[side];
  result.mid = 0.5 * (bounds[0] + bounds[1]);
  result.center = geom_center + result.mid * result.axis -
                  phase21_profile::kWheelRadius * result.radial;
  result.frame.col(0) = rolling;
  result.frame.col(1) = lateral;
  result.frame.col(2) = normal;
  return result;
}

Eigen::Vector3d radialVelocity(
    const ContactGeometry &geometry, const Eigen::Vector3d &axis_velocity) {
  const Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
  const double dot = geometry.axis.dot(normal);
  const double dot_velocity = axis_velocity.dot(normal);
  const double projection_velocity =
      -dot * dot_velocity / geometry.projection;
  const Eigen::Vector3d numerator = normal - dot * geometry.axis;
  const Eigen::Vector3d numerator_velocity =
      -dot_velocity * geometry.axis - dot * axis_velocity;
  return numerator_velocity / geometry.projection -
         numerator * projection_velocity /
             (geometry.projection * geometry.projection);
}

bool finite(const NominalWbcModel::Result &result) {
  if (!result.mass.allFinite() || !result.bias.allFinite() ||
      !result.actuation.allFinite() || !result.reduction.allFinite()) return false;
  for (int side = 0; side < 2; ++side) {
    if (!result.wrench_map[side].allFinite() ||
        !result.wrench_flu_map[side].allFinite() ||
        !result.contact_jacobian[side].allFinite() ||
        !result.contact_bias[side].allFinite() ||
        !result.contact_frame_world[side].allFinite() ||
        !result.contact_axis[side].allFinite() ||
        !result.point_force_wrench_projector[side].allFinite() ||
        !std::isfinite(result.wheel_position_b_x_m[side]) ||
        !std::isfinite(result.wheel_velocity_b_x_m_s[side]) ||
        !std::isfinite(result.wheel_position_b_z_m[side]) ||
        !std::isfinite(result.wheel_velocity_b_z_m_s[side]) ||
        !result.wheel_longitudinal_acceleration_map[side].allFinite() ||
        !std::isfinite(
            result.wheel_longitudinal_acceleration_bias_m_s2[side]) ||
        !result.wheel_vertical_acceleration_map[side].allFinite() ||
        !std::isfinite(
            result.wheel_vertical_acceleration_bias_m_s2[side]) ||
        !result.interaction_acceleration_map[side].allFinite() ||
        !result.interaction_contact_map[side].allFinite() ||
        !result.interaction_bias[side].allFinite()) return false;
  }
  return true;
}

}  // namespace

NominalWbcModel::Matrix6 pointContactWrenchProjector(
    const Eigen::Vector3d &contact_axis,
    const Eigen::Vector3d &contact_line_offset) {
  const Eigen::Vector3d axis = contact_axis.normalized();
  Eigen::Matrix<double, 6, 6> point_map;
  for (int point = 0; point < 2; ++point) {
    const Eigen::Vector3d lever = contact_line_offset +
        (point == 0 ? -0.5 : 0.5) * axis;
    point_map.block<3, 3>(0, 3 * point).setIdentity();
    point_map.block<3, 3>(3, 3 * point) <<
        0.0, -lever.z(), lever.y(),
        lever.z(), 0.0, -lever.x(),
        -lever.y(), lever.x(), 0.0;
  }
  const Eigen::JacobiSVD<NominalWbcModel::Matrix6> svd(
      point_map, Eigen::ComputeFullU);
  return svd.matrixU().leftCols<5>() * svd.matrixU().leftCols<5>().transpose();
}

NominalWbcModel::WorkspaceInspection NominalWbcModel::inspectWorkspace(
    const RobotState &state) {
  WorkspaceInspection inspection;
  double minimum_margin = std::numeric_limits<double>::infinity();
  for (int canonical = 0; canonical < 6; ++canonical) {
    auto &entry = inspection.joint[canonical];
    const int coordinate = canonical % 3;
    const bool wheel = coordinate == 2;
    entry.position_rad = state.joint_position_rad[canonical];
    entry.equilibrium_rad = phase21_profile::kCanonicalOffset[canonical] -
        phase21_profile::kEquilibriumActiveNative[canonical];
    entry.delta_rad = entry.position_rad - entry.equilibrium_rad;
    entry.lower_bound_rad = phase21_profile::kWorkspaceBounds[coordinate][0];
    entry.upper_bound_rad = phase21_profile::kWorkspaceBounds[coordinate][1];
    entry.lower_margin_rad = entry.delta_rad - entry.lower_bound_rad;
    entry.upper_margin_rad = entry.upper_bound_rad - entry.delta_rad;
    entry.signed_margin_rad =
        std::min(entry.lower_margin_rad, entry.upper_margin_rad);
    if (!wheel && entry.signed_margin_rad < minimum_margin) {
      minimum_margin = entry.signed_margin_rad;
      inspection.minimum_margin_index = canonical;
    }
    if (!wheel && inspection.first_failed_index < 0 &&
        entry.signed_margin_rad < 0.0) {
      inspection.first_failed_index = canonical;
    }
  }
  return inspection;
}

NominalWbcModel::Result NominalWbcModel::evaluate(
    const RobotState &state) const {
  Result result;
  if (validateRobotState(state) != ValidationError::kNone) return result;

  Eigen::Matrix<double, 10, 1> q =
      Eigen::Matrix<double, 10, 1>::Zero();
  const auto workspace = inspectWorkspace(state);
  if (!workspace.inside()) {
    result.status = Status::kOutsideWorkspace;
    return result;
  }
  for (int canonical = 0; canonical < 6; ++canonical) {
    const int native = phase21_profile::kActiveNative[canonical];
    q[native] = phase21_profile::kCanonicalOffset[canonical] -
                state.joint_position_rad[canonical];
  }
  for (int passive = 0; passive < 4; ++passive) {
    q[phase21_profile::kPassiveNative[passive]] =
        phase21_profile::kEquilibriumPassive[passive];
  }

  const Eigen::Vector3d base_position(
      state.base_position_n_m[0], state.base_position_n_m[1],
      state.base_position_n_m[2]);
  const Eigen::Matrix3d base_rotation = quaternion(state.q_n_from_b);
  const Vector16 zero = Vector16::Zero();
  Eigen::JacobiSVD<Eigen::Matrix<double, 6, 4>> passive_svd;
  for (int iteration = 0; iteration <= kMaximumReconstructionIterations;
       ++iteration) {
    const Kinematics bodies =
        forwardKinematics(base_position, base_rotation, q, zero);
    const auto residual = closureResidual(bodies);
    result.diagnostics.reconstruction_iterations = iteration;
    result.diagnostics.closure_residual_m = residual.cwiseAbs().maxCoeff();
    const auto jacobian = closureJacobian(bodies);
    Eigen::Matrix<double, 6, 4> passive;
    for (int column = 0; column < 4; ++column) {
      passive.col(column) =
          jacobian.col(6 + phase21_profile::kPassiveNative[column]);
    }
    passive_svd.compute(passive, Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto singular = passive_svd.singularValues();
    result.diagnostics.passive_minimum_singular_value = singular.minCoeff();
    result.diagnostics.passive_condition_number =
        singular.maxCoeff() / singular.minCoeff();
    if (result.diagnostics.closure_residual_m <= kClosureTolerance) break;
    if (iteration == kMaximumReconstructionIterations) {
      result.status = Status::kReconstructionFailure;
      return result;
    }
    Eigen::Vector4d step = passive_svd.solve(-residual);
    const double norm = step.norm();
    if (norm > 0.5) step *= 0.5 / norm;
    for (int index = 0; index < 4; ++index) {
      q[phase21_profile::kPassiveNative[index]] += step[index];
    }
  }
  if (result.diagnostics.passive_minimum_singular_value <
          kMinimumPassiveSingularValue ||
      result.diagnostics.passive_condition_number > kMaximumPassiveCondition) {
    result.status = Status::kIllConditioned;
    return result;
  }

  const Kinematics pose =
      forwardKinematics(base_position, base_rotation, q, zero);
  const auto closure_jacobian = closureJacobian(pose);
  Eigen::Matrix<double, 6, 4> passive;
  for (int column = 0; column < 4; ++column) {
    passive.col(column) = closure_jacobian.col(
        6 + phase21_profile::kPassiveNative[column]);
  }
  passive_svd.compute(passive, Eigen::ComputeFullU | Eigen::ComputeFullV);
  result.reduction.topLeftCorner<6, 6>().setIdentity();
  for (int canonical = 0; canonical < 6; ++canonical) {
    result.reduction(6 + phase21_profile::kActiveNative[canonical],
                     6 + canonical) = -1.0;
  }
  const Eigen::Matrix<double, 6, 12> known =
      closure_jacobian * result.reduction;
  const Eigen::Matrix<double, 4, 12> passive_reduction =
      passive_svd.solve(-known);
  for (int row = 0; row < 4; ++row) {
    result.reduction.row(6 + phase21_profile::kPassiveNative[row]) =
        passive_reduction.row(row);
  }

  Vector12 nu;
  nu.head<3>() << state.base_linear_velocity_n_m_s[0],
      state.base_linear_velocity_n_m_s[1],
      state.base_linear_velocity_n_m_s[2];
  nu.segment<3>(3) << state.base_angular_velocity_n_rad_s[0],
      state.base_angular_velocity_n_rad_s[1],
      state.base_angular_velocity_n_rad_s[2];
  for (int index = 0; index < 6; ++index) {
    nu[6 + index] = state.joint_velocity_rad_s[index];
  }
  const Vector16 tree_velocity = result.reduction * nu;
  const Kinematics bodies =
      forwardKinematics(base_position, base_rotation, q, tree_velocity);
  const Eigen::Vector4d passive_acceleration =
      passive_svd.solve(-closureBias(bodies));
  Vector16 reduction_bias = Vector16::Zero();
  for (int index = 0; index < 4; ++index) {
    reduction_bias[6 + phase21_profile::kPassiveNative[index]] =
        passive_acceleration[index];
  }
  const Kinematics reduced_bodies = forwardKinematics(
      base_position, base_rotation, q, tree_velocity, reduction_bias);
  result.reduction_bias = reduction_bias;

  Matrix16 tree_mass = Matrix16::Zero();
  Vector16 tree_bias = Vector16::Zero();
  const Eigen::Vector3d gravity(
      phase21_profile::kGravity[0], phase21_profile::kGravity[1],
      phase21_profile::kGravity[2]);
  for (int body_id = 1; body_id <= 11; ++body_id) {
    const auto &raw = phase21_profile::kBody[body_id - 1];
    const BodyState &body = bodies[body_id];
    const double mass = raw[7];
    const Eigen::Vector3d com_local = vector3(raw.data() + 8);
    const Eigen::Vector3d com_offset = body.rotation * com_local;
    const Matrix3x16 jv =
        body.linear_jacobian - skew(com_offset) * body.angular_jacobian;
    const Eigen::Matrix3d inertial_rotation =
        body.rotation * quaternion(raw.data() + 11);
    const Eigen::Matrix3d inertia = inertial_rotation *
        Eigen::Vector3d(raw[15], raw[16], raw[17]).asDiagonal() *
        inertial_rotation.transpose();
    tree_mass.noalias() += mass * jv.transpose() * jv +
                           body.angular_jacobian.transpose() * inertia *
                               body.angular_jacobian;
    const Eigen::Vector3d com_acceleration =
        body.linear_acceleration + body.angular_acceleration.cross(com_offset) +
        body.angular_velocity.cross(body.angular_velocity.cross(com_offset));
    tree_bias.noalias() +=
        jv.transpose() * (mass * (com_acceleration - gravity)) +
        body.angular_jacobian.transpose() *
            (inertia * body.angular_acceleration +
             body.angular_velocity.cross(inertia * body.angular_velocity));
  }
  result.mass.noalias() =
      result.reduction.transpose() * tree_mass * result.reduction;
  result.bias.noalias() = result.reduction.transpose() *
      (tree_bias + tree_mass * reduction_bias);
  for (int canonical = 0; canonical < 6; ++canonical) {
    const int native = phase21_profile::kActiveNative[canonical];
    result.actuation.col(canonical) =
        -result.reduction.row(6 + native).transpose();
  }

  for (int side = 0; side < 2; ++side) {
    const ContactGeometry geometry = contactGeometry(bodies, side);
    if (geometry.projection <= 1.0e-6) {
      result.status = Status::kIllConditioned;
      return result;
    }
    const BodyState &wheel = bodies[geometry.body_id];
    const BodyState &reduced_wheel = reduced_bodies[geometry.body_id];
    const Eigen::Vector3d wheel_relative_b =
        base_rotation.transpose() * (wheel.position - base_position);
    const Eigen::Vector3d base_velocity(
        state.base_linear_velocity_n_m_s[0],
        state.base_linear_velocity_n_m_s[1],
        state.base_linear_velocity_n_m_s[2]);
    const Eigen::Vector3d omega_b =
        base_rotation.transpose() * nu.segment<3>(3);
    const Eigen::Vector3d wheel_relative_velocity_b =
        base_rotation.transpose() * (wheel.linear_velocity - base_velocity) -
        omega_b.cross(wheel_relative_b);
    result.wheel_position_b_x_m[side] = wheel_relative_b.x();
    result.wheel_velocity_b_x_m_s[side] = wheel_relative_velocity_b.x();
    result.wheel_position_b_z_m[side] = wheel_relative_b.z();
    result.wheel_velocity_b_z_m_s[side] = wheel_relative_velocity_b.z();

    Eigen::Matrix<double, 3, 12> relative_acceleration_map =
        base_rotation.transpose() * wheel.linear_jacobian * result.reduction;
    relative_acceleration_map.leftCols<3>() -= base_rotation.transpose();
    relative_acceleration_map.middleCols<3>(3) +=
        skew(wheel_relative_b) * base_rotation.transpose();
    result.wheel_longitudinal_acceleration_map[side] =
        relative_acceleration_map.row(0);
    result.wheel_vertical_acceleration_map[side] =
        relative_acceleration_map.row(2);
    const Eigen::Vector3d relative_acceleration_bias_b =
        base_rotation.transpose() * reduced_wheel.linear_acceleration -
        2.0 * omega_b.cross(wheel_relative_velocity_b) -
        omega_b.cross(omega_b.cross(wheel_relative_b));
    result.wheel_vertical_acceleration_bias_m_s2[side] =
        relative_acceleration_bias_b.z();
    result.wheel_longitudinal_acceleration_bias_m_s2[side] =
        relative_acceleration_bias_b.x();
    const Eigen::Vector3d contact_local =
        wheel.rotation.transpose() * (geometry.center - wheel.position);
    const Matrix3x16 material_jacobian =
        pointJacobian(wheel, contact_local);
    Matrix3x16 geometric_jacobian;
    for (int column = 0; column < 16; ++column) {
      const Eigen::Vector3d axis_velocity =
          wheel.angular_jacobian.col(column).cross(geometry.axis);
      geometric_jacobian.col(column) =
          pointJacobian(wheel, geometry.geom_local).col(column) +
          geometry.mid * axis_velocity - phase21_profile::kWheelRadius *
              radialVelocity(geometry, axis_velocity);
    }
    result.contact_jacobian[side] =
        geometry.frame.transpose() * material_jacobian * result.reduction;
    const Eigen::Vector3d center_velocity =
        geometric_jacobian * tree_velocity;
    const Eigen::Vector3d offset = geometry.center - reduced_wheel.position;
    const Eigen::Vector3d material_velocity =
        reduced_wheel.linear_velocity + reduced_wheel.angular_velocity.cross(offset);
    const Eigen::Vector3d material_bias_world =
        reduced_wheel.linear_acceleration +
        reduced_wheel.angular_acceleration.cross(offset) +
        reduced_wheel.angular_velocity.cross(
            center_velocity - reduced_wheel.linear_velocity);
    const Eigen::Vector3d normal = Eigen::Vector3d::UnitZ();
    const Eigen::Vector3d axis_velocity =
        reduced_wheel.angular_velocity.cross(geometry.axis);
    const double dot = geometry.axis.dot(normal);
    const double projection_velocity =
        -dot * axis_velocity.dot(normal) / geometry.projection;
    const Eigen::Vector3d rolling_velocity =
        axis_velocity.cross(normal) / geometry.projection -
        geometry.frame.col(0) * projection_velocity / geometry.projection;
    Eigen::Matrix3d frame_velocity = Eigen::Matrix3d::Zero();
    frame_velocity.col(0) = rolling_velocity;
    frame_velocity.col(1) = normal.cross(rolling_velocity);
    result.contact_bias[side] = geometry.frame.transpose() *
        material_bias_world + frame_velocity.transpose() * material_velocity;
    result.contact_frame_world[side] = geometry.frame;
    result.contact_axis[side] = geometry.frame.transpose() * geometry.axis;
    const Eigen::Vector3d contact_line_offset(
        0.0, 0.0, kPhase46ActualContactLineNormalOffsetM[side]);
    result.point_force_wrench_projector[side] =
        pointContactWrenchProjector(result.contact_axis[side],
                                    contact_line_offset);
    const Matrix3x16 angular_jacobian = wheel.angular_jacobian;
    result.wrench_map[side].leftCols<3>() =
        result.reduction.transpose() * material_jacobian.transpose() *
        geometry.frame;
    result.wrench_map[side].rightCols<3>() =
        result.reduction.transpose() * angular_jacobian.transpose() *
        geometry.frame;
    const Eigen::Matrix3d frame_in_base = base_rotation.transpose() * geometry.frame;
    result.wrench_flu_map[side].setZero();
    result.wrench_flu_map[side].topLeftCorner<3, 3>() = frame_in_base;
    result.wrench_flu_map[side].bottomLeftCorner<3, 3>() =
        base_rotation.transpose() * skew(geometry.center - base_position) *
        geometry.frame;
    result.wrench_flu_map[side].bottomRightCorner<3, 3>() = frame_in_base;

    // The contact wrench acts on the wheel. Transport it once from the
    // contact point to the wheel-body origin, then express it in body/FLU.
    result.interaction_contact_map[side].setZero();
    result.interaction_contact_map[side].topLeftCorner<3, 3>() = frame_in_base;
    result.interaction_contact_map[side].bottomLeftCorner<3, 3>() =
        base_rotation.transpose() * skew(geometry.center - wheel.position) *
        geometry.frame;
    result.interaction_contact_map[side].bottomRightCorner<3, 3>() =
        frame_in_base;

    // Wheel free-body balance about the wheel-body origin O:
    // W_wheel_on_parent^O = W_contact_on_wheel^O - W_inertial,non-gravity^O.
    // At fixed q,nu the latter is affine in reduced acceleration nudot.
    const auto &raw = phase21_profile::kBody[geometry.body_id - 1];
    const double wheel_mass = raw[7];
    const Eigen::Vector3d wheel_com_local = vector3(raw.data() + 8);
    const Eigen::Vector3d wheel_com_offset =
        wheel.rotation * wheel_com_local;
    const Eigen::Matrix3d wheel_inertial_rotation =
        wheel.rotation * quaternion(raw.data() + 11);
    const Eigen::Matrix3d wheel_inertia_world =
        wheel_inertial_rotation *
        Eigen::Vector3d(raw[15], raw[16], raw[17]).asDiagonal() *
        wheel_inertial_rotation.transpose();
    const Matrix3x16 wheel_com_jacobian =
        pointJacobian(wheel, wheel_com_local);
    const Eigen::Matrix<double, 3, 12> force_acceleration_world =
        wheel_mass * wheel_com_jacobian * result.reduction;
    const Eigen::Matrix<double, 3, 12> moment_acceleration_world =
        wheel_inertia_world * wheel.angular_jacobian * result.reduction +
        skew(wheel_com_offset) * force_acceleration_world;
    result.interaction_acceleration_map[side].topRows<3>() =
        -base_rotation.transpose() * force_acceleration_world;
    result.interaction_acceleration_map[side].bottomRows<3>() =
        -base_rotation.transpose() * moment_acceleration_world;

    const Eigen::Vector3d wheel_com_acceleration =
        pointAcceleration(reduced_wheel, wheel_com_local);
    const Eigen::Vector3d inertial_force_world =
        wheel_mass * (wheel_com_acceleration - gravity);
    const Eigen::Vector3d inertial_moment_world =
        wheel_inertia_world * reduced_wheel.angular_acceleration +
        reduced_wheel.angular_velocity.cross(
            wheel_inertia_world * reduced_wheel.angular_velocity) +
        wheel_com_offset.cross(inertial_force_world);
    result.interaction_bias[side].head<3>() =
        -base_rotation.transpose() * inertial_force_world;
    result.interaction_bias[side].tail<3>() =
        -base_rotation.transpose() * inertial_moment_world;
  }
  result.native_joint_position_rad = q;
  if (!finite(result)) {
    result = Result{};
    result.status = Status::kNonFinite;
    return result;
  }
  result.status = Status::kOk;
  return result;
}

}  // namespace wheel_leg
