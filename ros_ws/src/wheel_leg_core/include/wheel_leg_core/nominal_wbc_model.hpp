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
  using Matrix1x12 = Eigen::Matrix<double, 1, kReducedDoF>;
  using Matrix16x12 = Eigen::Matrix<double, kTreeDoF, kReducedDoF>;
  using Matrix16 = Eigen::Matrix<double, kTreeDoF, kTreeDoF>;
  using Matrix16x6 = Eigen::Matrix<double, kTreeDoF, 6>;
  using Matrix6x16 = Eigen::Matrix<double, 6, kTreeDoF>;
  using Vector6 = Eigen::Matrix<double, 6, 1>;
  using Vector16 = Eigen::Matrix<double, kTreeDoF, 1>;

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

  struct WorkspaceEntry {
    double position_rad{0.0};
    double equilibrium_rad{0.0};
    double delta_rad{0.0};
    double lower_bound_rad{0.0};
    double upper_bound_rad{0.0};
    double lower_margin_rad{0.0};
    double upper_margin_rad{0.0};
    double signed_margin_rad{0.0};
  };

  struct WorkspaceInspection {
    std::array<WorkspaceEntry, 6> joint{};
    int minimum_margin_index{-1};
    int first_failed_index{-1};

    [[nodiscard]] bool inside() const { return first_failed_index < 0; }
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
    // Wheel axis expressed in the corresponding contact frame. The actual
    // two-point force image cannot produce a moment parallel to this axis.
    std::array<Eigen::Vector3d, 2> contact_axis{};
    // Orthogonal projector onto the actual Model-B two-point force image at
    // the production contact-wrench reference point.
    std::array<Matrix6, 2> point_force_wrench_projector{};
    // Wheel-body origin relative to the canonical base-control point,
    // expressed in controller body/FLU. Side order is left, right.
    std::array<double, 2> wheel_position_b_x_m{};
    std::array<double, 2> wheel_velocity_b_x_m_s{};
    // Relative wheel-origin longitudinal acceleration in body/FLU:
    //   ddxi = A_xi * nudot + b_xi.
    std::array<Matrix1x12, 2> wheel_longitudinal_acceleration_map{};
    std::array<double, 2> wheel_longitudinal_acceleration_bias_m_s2{};
    std::array<double, 2> wheel_position_b_z_m{};
    std::array<double, 2> wheel_velocity_b_z_m_s{};
    // Relative wheel-origin vertical acceleration in body/FLU is affine in
    // reduced generalized acceleration:
    //   ddzeta = A_zeta * nudot + b_zeta.
    std::array<Matrix1x12, 2> wheel_vertical_acceleration_map{};
    std::array<double, 2> wheel_vertical_acceleration_bias_m_s2{};
    // Internal wrench exerted by the wheel follower on the leg/base at the
    // wheel-body origin, expressed in body/FLU:
    //   W_I = A_nudot * nudot + A_contact * w_C + b_I.
    // Each contact wrench is contact-centred and expressed in its contact
    // frame; wrench component order is [Fx,Fy,Fz,Tx,Ty,Tz].
    std::array<Matrix6x12, 2> interaction_acceleration_map{};
    std::array<Matrix6, 2> interaction_contact_map{};
    std::array<Eigen::Matrix<double, 6, 1>, 2> interaction_bias{};
    Matrix16x12 reduction{Matrix16x12::Zero()};
    // At the evaluated state, native tree acceleration is affine in the
    // reduced acceleration: qdd_tree = reduction * nudot + reduction_bias.
    Vector16 reduction_bias{Vector16::Zero()};
    // Read-only audit provenance for the affine closure relation
    // J_eq * qdd_tree + JdotV = 0 used to construct the reduction.
    Matrix6x16 equality_jacobian{Matrix6x16::Zero()};
    Vector6 equality_jdot_v{Vector6::Zero()};
    Matrix16 full_mass{Matrix16::Zero()};
    Vector16 full_bias{Vector16::Zero()};
    Matrix16x6 full_actuation{Matrix16x6::Zero()};
    std::array<Matrix16x6, 2> full_wrench_map{};
    Eigen::Matrix<double, 10, 1> native_joint_position_rad{
        Eigen::Matrix<double, 10, 1>::Zero()};

    [[nodiscard]] bool ok() const { return status == Status::kOk; }
  };

  [[nodiscard]] static WorkspaceInspection inspectWorkspace(
      const RobotState &state);
  [[nodiscard]] Result evaluate(const RobotState &state) const;
};

[[nodiscard]] NominalWbcModel::Matrix6 pointContactWrenchProjector(
    const Eigen::Vector3d &contact_axis,
    const Eigen::Vector3d &contact_line_offset);

}  // namespace wheel_leg
