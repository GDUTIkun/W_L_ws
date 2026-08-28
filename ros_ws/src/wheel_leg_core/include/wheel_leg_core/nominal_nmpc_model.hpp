#pragma once

#include <Eigen/Core>

namespace wheel_leg {

class NominalNmpcModel {
 public:
  static constexpr int kStateSize = 12;
  static constexpr int kInputSize = 12;
  using State = Eigen::Matrix<double, kStateSize, 1>;
  using Input = Eigen::Matrix<double, kInputSize, 1>;
  using StateJacobian = Eigen::Matrix<double, kStateSize, kStateSize>;
  using InputJacobian = Eigen::Matrix<double, kStateSize, kInputSize>;

  enum class Status { kOk, kInvalidInput, kOutsideChart };

  struct Result {
    Status status{Status::kInvalidInput};
    State continuous{State::Zero()};
    State next{State::Zero()};
    StateJacobian continuous_state_jacobian{StateJacobian::Zero()};
    InputJacobian continuous_input_jacobian{InputJacobian::Zero()};
    StateJacobian discrete_state_jacobian{StateJacobian::Zero()};
    InputJacobian discrete_input_jacobian{InputJacobian::Zero()};

    [[nodiscard]] bool ok() const { return status == Status::kOk; }
  };

  [[nodiscard]] Result evaluate(
      const State &state, const Input &input,
      const Eigen::Matrix3d &reference_rotation_n_from_b =
          Eigen::Matrix3d::Identity()) const;
};

}  // namespace wheel_leg
