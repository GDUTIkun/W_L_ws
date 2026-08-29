#pragma once

#include <Eigen/Core>

#include <array>

#include "wheel_leg_core/types.hpp"

namespace wheel_leg {

class NominalWbcModel {
 public:
  static constexpr int kReducedDoF = 12;
  static constexpr int kTreeDoF = 16;

  using Vector12 = Eigen::Matrix<double, kReducedDoF, 1>;
  using Matrix12 = Eigen::Matrix<double, kReducedDoF, kReducedDoF>;
  using Matrix12x6 = Eigen::Matrix<double, kReducedDoF, 6>;
  using Matrix6x12 = Eigen::Matrix<double, 6, kReducedDoF>;
  using Matrix6 = Eigen::Matrix<double, 6, 6>;
  using Matrix3x12 = Eigen::Matrix<double, 3, kReducedDoF>;
  using Matrix16x12 = Eigen::Matrix<double, kTreeDoF, kReducedDoF>;

  enum class Status {
    kOk,
    kInvalidState,
    kOutsideWorkspace,
    kReconstructionFailure,
    kIllConditioned,
    kNonFinite,
  };

  struct Diagnostics {
    int reconstruction_iterations{0};
    double closure_residual_m{0.0};
    double passive_minimum_singular_value{0.0};
    double passive_condition_number{0.0};
  };

  struct Result {
    Status status{Status::kInvalidState};
    Diagnostics diagnostics{};
    Matrix12 mass{Matrix12::Zero()};
    Vector12 bias{Vector12::Zero()};
    Matrix12x6 actuation{Matrix12x6::Zero()};
    std::array<Matrix12x6, 2> wrench_map{};
    std::array<Matrix6, 2> wrench_flu_map{};
    std::array<Matrix3x12, 2> contact_jacobian{};
    std::array<Eigen::Vector3d, 2> contact_bias{};
    std::array<Eigen::Matrix3d, 2> contact_frame_world{};
    // Wheel-body origin relative to the canonical base-control point,
    // expressed in controller body/FLU. Side order is left, right.
    std::array<double, 2> wheel_position_b_x_m{};
    std::array<double, 2> wheel_velocity_b_x_m_s{};
    // Internal wrench exerted by the wheel follower on the leg/base at the
    // wheel-body origin, expressed in body/FLU:
    //   W_I = A_nudot * nudot + A_contact * w_C + b_I.
    // Each contact wrench is contact-centred and expressed in its contact
    // frame; wrench component order is [Fx,Fy,Fz,Tx,Ty,Tz].
    std::array<Matrix6x12, 2> interaction_acceleration_map{};
    std::array<Matrix6, 2> interaction_contact_map{};
    std::array<Eigen::Matrix<double, 6, 1>, 2> interaction_bias{};
    Matrix16x12 reduction{Matrix16x12::Zero()};
    Eigen::Matrix<double, 10, 1> native_joint_position_rad{
        Eigen::Matrix<double, 10, 1>::Zero()};

    [[nodiscard]] bool ok() const { return status == Status::kOk; }
  };

  [[nodiscard]] Result evaluate(const RobotState &state) const;
};

}  // namespace wheel_leg
