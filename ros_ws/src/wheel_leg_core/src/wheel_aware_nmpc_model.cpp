#include "wheel_leg_core/wheel_aware_nmpc_model.hpp"

#include <cmath>

#include <Eigen/Geometry>
#include <Eigen/LU>
#include <unsupported/Eigen/AutoDiff>

namespace wheel_leg {
namespace {

constexpr double kBodyMassKg = 5.7482000000000006;
constexpr double kWheelMassKg = 0.34310000000000002;
constexpr double kWheelRadiusM = 0.05;
constexpr double kWheelAxleInertiaKgM2 = 0.00042373759089541461;
constexpr double kWheelDenominatorKgM =
    kWheelMassKg * kWheelRadiusM +
    kWheelAxleInertiaKgM2 / kWheelRadiusM;
constexpr double kGravityMps2 = 9.81;
constexpr double kStepS = 0.02;
constexpr double kMaximumChartNormRad = 0.35;
constexpr double kLeftXiMinimumM = -0.3303432354;
constexpr double kLeftXiMaximumM = 0.1678677251;
constexpr double kRightXiMinimumM = -0.3321211483;
constexpr double kRightXiMaximumM = 0.1659029424;
constexpr int kDerivativeSize =
    WheelAwareNmpcModel::kStateSize + WheelAwareNmpcModel::kInputSize;

template <typename Scalar>
using Vector3 = Eigen::Matrix<Scalar, 3, 1>;

template <typename Scalar>
using Matrix3 = Eigen::Matrix<Scalar, 3, 3>;

template <typename Scalar>
using ModelState = Eigen::Matrix<Scalar, WheelAwareNmpcModel::kStateSize, 1>;

template <typename Scalar>
using ModelInput = Eigen::Matrix<Scalar, WheelAwareNmpcModel::kInputSize, 1>;

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
  result << Scalar(0), -vector.z(), vector.y(), vector.z(), Scalar(0),
      -vector.x(), -vector.y(), vector.x(), Scalar(0);
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
      -0.011186360321930223, 0.00010351112192572815,
      -0.050073820064730427).finished().template cast<Scalar>();
  Matrix3<double> inertia_values;
  inertia_values <<
      0.14032539391425894, -0.00027482615417932365,
      -0.0055301280789718825, -0.00027482615417932365,
      0.075346414957965305, 0.00019749711948992591,
      -0.0055301280789718825, 0.00019749711948992591,
      0.094068657813524068;
  const Matrix3<Scalar> inertia_com_b = inertia_values.template cast<Scalar>();

  const Vector3<Scalar> left_force_b = input.template segment<3>(0);
  const Vector3<Scalar> left_torque_b = input.template segment<3>(3);
  const Vector3<Scalar> right_force_b = input.template segment<3>(6);
  const Vector3<Scalar> right_torque_b = input.template segment<3>(9);
  const Vector3<Scalar> left_origin_b = (Vector3<Scalar>() <<
      state[12], Scalar(0.21229919000000008),
      Scalar(-0.26587051502608744)).finished();
  const Vector3<Scalar> right_origin_b = (Vector3<Scalar>() <<
      state[13], Scalar(-0.21230080999999992),
      Scalar(-0.26574406892872388)).finished();
  const Vector3<Scalar> force_b = left_force_b + right_force_b;
  const Vector3<Scalar> moment_b =
      left_torque_b + left_origin_b.cross(left_force_b) +
      right_torque_b + right_origin_b.cross(right_force_b);
  const Vector3<Scalar> force_n = rotation * force_b;
  const Vector3<Scalar> moment_b_n = rotation * moment_b;
  const Vector3<Scalar> com_offset_n = rotation * com_b;
  const Matrix3<Scalar> inertia_com_n =
      rotation * inertia_com_b * rotation.transpose();
  const Vector3<Scalar> moment_com_n =
      moment_b_n - com_offset_n.cross(force_n);
  const Vector3<Scalar> angular_acceleration = inertia_com_n.inverse() *
      (moment_com_n -
       angular_velocity.cross(inertia_com_n * angular_velocity));
  Vector3<Scalar> com_acceleration = force_n / Scalar(kBodyMassKg);
  com_acceleration.z() -= Scalar(kGravityMps2);
  const Vector3<Scalar> base_acceleration =
      com_acceleration - angular_acceleration.cross(com_offset_n) -
      angular_velocity.cross(angular_velocity.cross(com_offset_n));

  const Scalar base_forward_acceleration = force_b.x() / Scalar(kBodyMassKg);
  const Scalar left_wheel_acceleration =
      -base_forward_acceleration -
      (Scalar(kWheelRadiusM) * left_force_b.x() + left_torque_b.y()) /
          Scalar(kWheelDenominatorKgM);
  const Scalar right_wheel_acceleration =
      -base_forward_acceleration -
      (Scalar(kWheelRadiusM) * right_force_b.x() + right_torque_b.y()) /
          Scalar(kWheelDenominatorKgM);

  ModelState<Scalar> result;
  result.template segment<3>(0) = linear_velocity;
  result.template segment<3>(3) =
      leftJacobianInverse(rotation_vector) * angular_velocity;
  result.template segment<3>(6) = base_acceleration;
  result.template segment<3>(9) = angular_acceleration;
  result[12] = state[14];
  result[13] = state[15];
  result[14] = left_wheel_acceleration;
  result[15] = right_wheel_acceleration;
  return result;
}

template <typename Scalar>
ModelState<Scalar> rk4Step(
    const ModelState<Scalar> &state, const ModelInput<Scalar> &input,
    const Matrix3<Scalar> &reference_rotation, double step_s) {
  const ModelState<Scalar> k1 = flow(state, input, reference_rotation);
  const ModelState<Scalar> stage2 = state + Scalar(0.5 * step_s) * k1;
  const ModelState<Scalar> k2 = flow(stage2, input, reference_rotation);
  const ModelState<Scalar> stage3 = state + Scalar(0.5 * step_s) * k2;
  const ModelState<Scalar> k3 = flow(stage3, input, reference_rotation);
  const ModelState<Scalar> stage4 = state + Scalar(step_s) * k3;
  const ModelState<Scalar> k4 = flow(stage4, input, reference_rotation);
  return state + Scalar(step_s / 6.0) *
      (k1 + Scalar(2) * k2 + Scalar(2) * k3 + k4);
}

template <typename Scalar>
ModelState<Scalar> rk4(
    const ModelState<Scalar> &state, const ModelInput<Scalar> &input,
    const Matrix3<Scalar> &reference_rotation) {
  constexpr double kSubstepS = 0.5 * kStepS;
  return rk4Step(
      rk4Step(state, input, reference_rotation, kSubstepS), input,
      reference_rotation, kSubstepS);
}

}  // namespace

WheelAwareNmpcModel::Result WheelAwareNmpcModel::evaluate(
    const State &state, const Input &input,
    const Eigen::Matrix3d &reference_rotation_n_from_b) const {
  Result result;
  if (!state.allFinite() || !input.allFinite() ||
      !reference_rotation_n_from_b.allFinite() ||
      (reference_rotation_n_from_b.transpose() * reference_rotation_n_from_b -
       Eigen::Matrix3d::Identity()).cwiseAbs().maxCoeff() > 1.0e-9 ||
      std::abs(reference_rotation_n_from_b.determinant() - 1.0) > 1.0e-9) {
    return result;
  }
  if (state.segment<3>(3).norm() > kMaximumChartNormRad) {
    result.status = Status::kOutsideChart;
    return result;
  }
  if (state[12] < kLeftXiMinimumM || state[12] > kLeftXiMaximumM ||
      state[13] < kRightXiMinimumM || state[13] > kRightXiMaximumM) {
    result.status = Status::kOutsideWheelWorkspace;
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
  const ModelState<AutoDiff> continuous = flow(
      differentiated_state, differentiated_input, differentiated_reference);
  const ModelState<AutoDiff> next = rk4(
      differentiated_state, differentiated_input, differentiated_reference);
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
