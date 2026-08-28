#include "wheel_leg_core/nominal_nmpc_model.hpp"

#include <cmath>

#include <Eigen/LU>
#include <Eigen/Geometry>
#include <unsupported/Eigen/AutoDiff>

namespace wheel_leg {
namespace {

constexpr double kMassKg = 6.4344;
constexpr double kGravityMps2 = 9.81;
constexpr double kStepS = 0.02;
constexpr double kMaximumChartNormRad = 0.35;
constexpr int kDerivativeSize =
    NominalNmpcModel::kStateSize + NominalNmpcModel::kInputSize;

template <typename Scalar>
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

template <typename Scalar>
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

template <typename Scalar>
using ModelState = Eigen::Matrix<Scalar, NominalNmpcModel::kStateSize, 1>;

template <typename Scalar>
using ModelInput = Eigen::Matrix<Scalar, NominalNmpcModel::kInputSize, 1>;

template <typename Scalar>
double valueOf(const Scalar &value) {
  return static_cast<double>(value);
}

template <typename Derivatives>
double valueOf(const Eigen::AutoDiffScalar<Derivatives> &value) {
  return value.value();
}

template <typename Scalar>
Matrix3<Scalar> skew(const Vector3<Scalar> &vector) {
  Matrix3<Scalar> result;
  result << Scalar(0), -vector.z(), vector.y(),
      vector.z(), Scalar(0), -vector.x(),
      -vector.y(), vector.x(), Scalar(0);
  return result;
}

template <typename Scalar>
Matrix3<Scalar> rotationMatrix(const Vector3<Scalar> &vector) {
  using Eigen::numext::cos;
  using Eigen::numext::sin;
  using Eigen::numext::sqrt;
  const Scalar squared_angle = vector.squaredNorm();
  const Matrix3<Scalar> hat = skew(vector);
  Scalar sine_scale;
  Scalar cosine_scale;
  if (valueOf(squared_angle) < 1.0e-12) {
    sine_scale = Scalar(1) - squared_angle / Scalar(6) +
                 squared_angle * squared_angle / Scalar(120);
    cosine_scale = Scalar(0.5) - squared_angle / Scalar(24) +
                   squared_angle * squared_angle / Scalar(720);
  } else {
    const Scalar angle = sqrt(squared_angle);
    sine_scale = sin(angle) / angle;
    cosine_scale = (Scalar(1) - cos(angle)) / squared_angle;
  }
  return Matrix3<Scalar>::Identity() + sine_scale * hat +
         cosine_scale * hat * hat;
}

template <typename Scalar>
Matrix3<Scalar> leftJacobianInverse(const Vector3<Scalar> &vector) {
  using Eigen::numext::cos;
  using Eigen::numext::sin;
  using Eigen::numext::sqrt;
  const Scalar squared_angle = vector.squaredNorm();
  const Matrix3<Scalar> hat = skew(vector);
  Scalar coefficient;
  if (valueOf(squared_angle) < 1.0e-12) {
    coefficient = Scalar(1.0 / 12.0) + squared_angle / Scalar(720);
  } else {
    const Scalar angle = sqrt(squared_angle);
    coefficient = Scalar(1) / squared_angle -
                  (Scalar(1) + cos(angle)) /
                      (Scalar(2) * angle * sin(angle));
  }
  return Matrix3<Scalar>::Identity() - Scalar(0.5) * hat +
         coefficient * hat * hat;
}

template <typename Scalar>
ModelState<Scalar> flow(
    const ModelState<Scalar> &state, const ModelInput<Scalar> &input,
    const Matrix3<Scalar> &reference_rotation) {
  const Vector3<Scalar> rotation_vector = state.template segment<3>(3);
  const Vector3<Scalar> linear_velocity = state.template segment<3>(6);
  const Vector3<Scalar> angular_velocity = state.template segment<3>(9);
  const Matrix3<Scalar> rotation =
      rotationMatrix(rotation_vector) * reference_rotation;
  const Vector3<Scalar> com_b = (Vector3<double>() <<
      -0.011180287402107112, 9.238574064202139e-05,
      -0.07308450550529051).finished().template cast<Scalar>();
  Matrix3<double> inertia_values;
  inertia_values <<
      0.2024829571803848, -0.0005268982127304279, -0.005522605148090465,
      -0.000526898212730428, 0.10473543935656524, 0.00018285882563471784,
      -0.005522605148090467, 0.0001828588256347179, 0.12768847489813864;
  const Matrix3<Scalar> inertia_com_b = inertia_values.template cast<Scalar>();
  const Vector3<Scalar> force_n = rotation *
      (input.template segment<3>(0) + input.template segment<3>(6));
  const Vector3<Scalar> moment_b_n = rotation *
      (input.template segment<3>(3) + input.template segment<3>(9));
  const Vector3<Scalar> com_offset_n = rotation * com_b;
  const Matrix3<Scalar> inertia_com_n =
      rotation * inertia_com_b * rotation.transpose();
  const Vector3<Scalar> moment_com_n =
      moment_b_n - com_offset_n.cross(force_n);
  const Vector3<Scalar> angular_acceleration = inertia_com_n.inverse() *
      (moment_com_n -
       angular_velocity.cross(inertia_com_n * angular_velocity));
  Vector3<Scalar> com_acceleration = force_n / Scalar(kMassKg);
  com_acceleration.z() -= Scalar(kGravityMps2);
  const Vector3<Scalar> base_acceleration =
      com_acceleration - angular_acceleration.cross(com_offset_n) -
      angular_velocity.cross(angular_velocity.cross(com_offset_n));

  ModelState<Scalar> result;
  result.template segment<3>(0) = linear_velocity;
  result.template segment<3>(3) =
      leftJacobianInverse(rotation_vector) * angular_velocity;
  result.template segment<3>(6) = base_acceleration;
  result.template segment<3>(9) = angular_acceleration;
  return result;
}

template <typename Scalar>
ModelState<Scalar> rk4(
    const ModelState<Scalar> &state, const ModelInput<Scalar> &input,
    const Matrix3<Scalar> &reference_rotation) {
  const ModelState<Scalar> k1 = flow(state, input, reference_rotation);
  const ModelState<Scalar> stage2 = state + Scalar(0.5 * kStepS) * k1;
  const ModelState<Scalar> k2 = flow(stage2, input, reference_rotation);
  const ModelState<Scalar> stage3 = state + Scalar(0.5 * kStepS) * k2;
  const ModelState<Scalar> k3 = flow(stage3, input, reference_rotation);
  const ModelState<Scalar> stage4 = state + Scalar(kStepS) * k3;
  const ModelState<Scalar> k4 = flow(stage4, input, reference_rotation);
  return state + Scalar(kStepS / 6.0) *
      (k1 + Scalar(2) * k2 + Scalar(2) * k3 + k4);
}

}  // namespace

NominalNmpcModel::Result NominalNmpcModel::evaluate(
    const State &state, const Input &input,
    const Eigen::Matrix3d &reference_rotation_n_from_b) const {
  Result result;
  if (!state.allFinite() || !input.allFinite() ||
      !reference_rotation_n_from_b.allFinite() ||
      (reference_rotation_n_from_b.transpose() * reference_rotation_n_from_b -
       Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() > 1.0e-9 ||
      std::abs(reference_rotation_n_from_b.determinant() - 1.0) > 1.0e-9) {
    result.status = Status::kInvalidInput;
    return result;
  }
  if (state.segment<3>(3).norm() > kMaximumChartNormRad) {
    result.status = Status::kOutsideChart;
    return result;
  }

  using Derivatives = Eigen::Matrix<double, kDerivativeSize, 1>;
  using AutoDiff = Eigen::AutoDiffScalar<Derivatives>;
  const Matrix3<AutoDiff> differentiated_reference =
      reference_rotation_n_from_b.cast<AutoDiff>();
  ModelState<AutoDiff> differentiated_state;
  ModelInput<AutoDiff> differentiated_input;
  for (int row = 0; row < kStateSize; ++row) {
    differentiated_state[row].value() = state[row];
    differentiated_state[row].derivatives().setZero();
    differentiated_state[row].derivatives()[row] = 1.0;
  }
  for (int row = 0; row < kInputSize; ++row) {
    differentiated_input[row].value() = input[row];
    differentiated_input[row].derivatives().setZero();
    differentiated_input[row].derivatives()[kStateSize + row] = 1.0;
  }
  const ModelState<AutoDiff> continuous =
      flow(differentiated_state, differentiated_input,
           differentiated_reference);
  const ModelState<AutoDiff> next =
      rk4(differentiated_state, differentiated_input,
          differentiated_reference);
  for (int row = 0; row < kStateSize; ++row) {
    result.continuous[row] = continuous[row].value();
    result.next[row] = next[row].value();
    result.continuous_state_jacobian.row(row) =
        continuous[row].derivatives().head<kStateSize>().transpose();
    result.continuous_input_jacobian.row(row) =
        continuous[row].derivatives().tail<kInputSize>().transpose();
    result.discrete_state_jacobian.row(row) =
        next[row].derivatives().head<kStateSize>().transpose();
    result.discrete_input_jacobian.row(row) =
        next[row].derivatives().tail<kInputSize>().transpose();
  }
  if (!result.continuous.allFinite() || !result.next.allFinite() ||
      !result.continuous_state_jacobian.allFinite() ||
      !result.continuous_input_jacobian.allFinite() ||
      !result.discrete_state_jacobian.allFinite() ||
      !result.discrete_input_jacobian.allFinite()) {
    return Result{};
  }
  result.status = Status::kOk;
  return result;
}

}  // namespace wheel_leg
