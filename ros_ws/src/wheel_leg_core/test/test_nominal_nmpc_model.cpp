#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

#include <Eigen/Geometry>

#include "wheel_leg_core/nominal_nmpc_model.hpp"

namespace {

template <typename Matrix>
void readMatrix(std::istream &stream, Matrix &matrix) {
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      if (!(stream >> matrix(row, column))) std::abort();
    }
  }
}

template <typename Matrix>
double maximumError(const Matrix &actual, const Matrix &expected) {
  return (actual - expected).cwiseAbs().maxCoeff();
}

}  // namespace

int main() {
  using wheel_leg::NominalNmpcModel;
  std::ifstream golden(NMPC_MODEL_GOLDEN_PATH);
  if (!golden.good()) return 1;
  int sample_count = 0;
  if (!(golden >> sample_count) || sample_count != 5) return 1;
  NominalNmpcModel model;
  double maximum_continuous = 0.0;
  double maximum_next = 0.0;
  double maximum_continuous_jacobian = 0.0;
  double maximum_discrete_jacobian = 0.0;
  for (int sample = 0; sample < sample_count; ++sample) {
    std::string id;
    Eigen::Vector3d reference_rotation_vector;
    NominalNmpcModel::State state;
    NominalNmpcModel::Input input;
    NominalNmpcModel::State continuous;
    NominalNmpcModel::State next;
    NominalNmpcModel::StateJacobian continuous_a;
    NominalNmpcModel::InputJacobian continuous_b;
    NominalNmpcModel::StateJacobian discrete_a;
    NominalNmpcModel::InputJacobian discrete_b;
    if (!(golden >> id)) return 1;
    readMatrix(golden, reference_rotation_vector);
    readMatrix(golden, state);
    readMatrix(golden, input);
    readMatrix(golden, continuous);
    readMatrix(golden, next);
    readMatrix(golden, continuous_a);
    readMatrix(golden, continuous_b);
    readMatrix(golden, discrete_a);
    readMatrix(golden, discrete_b);
    const double reference_angle = reference_rotation_vector.norm();
    const Eigen::Matrix3d reference_rotation =
        reference_angle == 0.0
            ? Eigen::Matrix3d::Identity()
            : Eigen::AngleAxisd(
                  reference_angle,
                  reference_rotation_vector / reference_angle).toRotationMatrix();
    const auto actual = model.evaluate(state, input, reference_rotation);
    maximum_continuous = std::max(
        maximum_continuous, maximumError(actual.continuous, continuous));
    maximum_next = std::max(maximum_next, maximumError(actual.next, next));
    maximum_continuous_jacobian = std::max({
        maximum_continuous_jacobian,
        maximumError(actual.continuous_state_jacobian, continuous_a),
        maximumError(actual.continuous_input_jacobian, continuous_b)});
    maximum_discrete_jacobian = std::max({
        maximum_discrete_jacobian,
        maximumError(actual.discrete_state_jacobian, discrete_a),
        maximumError(actual.discrete_input_jacobian, discrete_b)});
    if (!actual.ok() ||
        maximumError(actual.continuous, continuous) > 2.0e-8 ||
        maximumError(actual.next, next) > 2.0e-8 ||
        maximumError(actual.continuous_state_jacobian, continuous_a) > 1.0e-5 ||
        maximumError(actual.continuous_input_jacobian, continuous_b) > 1.0e-5 ||
        maximumError(actual.discrete_state_jacobian, discrete_a) > 1.0e-5 ||
        maximumError(actual.discrete_input_jacobian, discrete_b) > 1.0e-5) {
      return 1;
    }
  }

  NominalNmpcModel::State invalid = NominalNmpcModel::State::Zero();
  NominalNmpcModel::Input input = NominalNmpcModel::Input::Zero();
  invalid[3] = 0.351;
  if (model.evaluate(invalid, input).status !=
      NominalNmpcModel::Status::kOutsideChart) return 1;
  invalid.setZero();
  invalid[0] = std::numeric_limits<double>::quiet_NaN();
  if (model.evaluate(invalid, input).status !=
      NominalNmpcModel::Status::kInvalidInput) return 1;
  std::cout << "continuous=" << maximum_continuous
            << " next=" << maximum_next
            << " continuous_jacobian=" << maximum_continuous_jacobian
            << " discrete_jacobian=" << maximum_discrete_jacobian << '\n';
}
