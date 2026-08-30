#pragma once

#include "wheel_leg_core/dense_qp_solver.hpp"
#include "wheel_leg_core/weighted_wbc_problem.hpp"

namespace wheel_leg {

class WeightedWbcController {
 public:
  enum class Task : std::size_t {
    kContact, kBaseX, kHeight, kOrientation, kLeg, kWheelVerticalManifold,
    kWheelLongitudinalTracking,
    kNativeWheelRate,
    kWrenchFidelity, kSlackPenalty, kCount,
  };
  static constexpr std::size_t kTaskCount = static_cast<std::size_t>(Task::kCount);
  enum class Status {
    kOk,
    kModelRejected,
    kProblemRejected,
    kSolverRejected,
    kHardViolation,
    kNonFinite,
  };

  struct Result {
    Status status{Status::kProblemRejected};
    NominalWbcModel::Status model_status{NominalWbcModel::Status::kInvalidState};
    DenseQpSolver::Status solver_status{DenseQpSolver::Status::kInvalidInput};
    JointVector torque_nm{};
    int iterations{0};
    double hard_violation{0.0};
    double stationarity_residual{0.0};
    double primal_residual{0.0};
    double dual_residual{0.0};
    NominalWbcModel::Diagnostics model_diagnostics{};
    Eigen::Matrix<double, 42, 1> physical_solution{
        Eigen::Matrix<double, 42, 1>::Zero()};
    std::array<double, kTaskCount> task_max_abs_normalized_residual{};
    std::array<double, kTaskCount> task_normalized_squared_cost{};
    double maximum_normalized_slack{0.0};
    Eigen::Matrix<double, 12, 1> realized_interaction_wrench_flu{
        Eigen::Matrix<double, 12, 1>::Zero()};
    Eigen::Matrix<double, 12, 1> interaction_wrench_residual_flu{
        Eigen::Matrix<double, 12, 1>::Zero()};
    Eigen::Matrix<double, 12, 1> signed_interaction_slack_flu{
        Eigen::Matrix<double, 12, 1>::Zero()};
    Eigen::Vector2d wheel_position_b_z_m{Eigen::Vector2d::Zero()};
    Eigen::Vector2d wheel_velocity_b_z_m_s{Eigen::Vector2d::Zero()};
    Eigen::Vector2d wheel_vertical_acceleration_m_s2{Eigen::Vector2d::Zero()};
    Eigen::Vector2d wheel_position_b_x_m{Eigen::Vector2d::Zero()};
    Eigen::Vector2d wheel_velocity_b_x_m_s{Eigen::Vector2d::Zero()};
    Eigen::Vector2d wheel_longitudinal_acceleration_m_s2{Eigen::Vector2d::Zero()};
    NominalWbcModel::Matrix16x12 reduction{
        NominalWbcModel::Matrix16x12::Zero()};
    NominalWbcModel::Vector16 reduction_bias{
        NominalWbcModel::Vector16::Zero()};
    std::array<NominalWbcModel::Matrix12x6, 2> contact_wrench_map{};
    std::array<NominalWbcModel::Matrix1x12, 2>
        wheel_longitudinal_acceleration_map{};
    Eigen::Vector2d wheel_longitudinal_acceleration_bias_m_s2{
        Eigen::Vector2d::Zero()};
    Eigen::Matrix<double, 6, 1> contact_task_residual{
        Eigen::Matrix<double, 6, 1>::Zero()};
    std::array<int, 3> active_inequality_count{};
    std::array<double, 3> minimum_inequality_margin{};

    [[nodiscard]] bool ok() const { return status == Status::kOk; }
  };

  explicit WeightedWbcController(
      WeightedWbcProfile profile = WeightedWbcProfile::kNominal);
  void reset();
  [[nodiscard]] Result step(const RobotState &state,
                            const WbcReference &reference);

 private:
  NominalWbcModel model_{};
  WeightedWbcProblem problem_{};
  DenseQpSolver solver_;
  WeightedWbcProfile profile_{WeightedWbcProfile::kNominal};
  bool warm_start_available_{false};
};

}  // namespace wheel_leg
