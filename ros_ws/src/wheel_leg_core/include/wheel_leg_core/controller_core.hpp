#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "wheel_leg_core/types.hpp"
#include "wheel_leg_core/nominal_nmpc_solver.hpp"
#include "wheel_leg_core/weighted_wbc_controller.hpp"
#include "wheel_leg_core/wheel_aware_nmpc_solver.hpp"
#include "wheel_leg_core/wheel_position_planner.hpp"

namespace wheel_leg {

enum class ControllerMode {
  kZero,
  kJointPdGravity,
  kSimpleStanding,
  kSimpleStanding3d,
  kWeightedWbc,
  kNominalNmpcWbc,
  kPhase27MinimalNmpcWbc,
};

enum class NmpcReferenceProfile {
  kHold,
  kPositiveStep,
  kNegativeStep,
  kStepReturn,
};

enum class NmpcFaultInjection {
  kNone,
  kSolverFailure,
  kLate,
  kStale,
  kNonFinite,
};

struct JointReference {
  JointVector position_rad{};
  JointVector velocity_rad_s{};
};

struct GravityHarmonic {
  std::array<int, 3> native_wave_number{};
  double sin_torque_nm{0.0};
  double cos_torque_nm{0.0};
};

struct LegGravityProfile {
  std::array<double, 3> canonical_offset_rad{};
  std::array<GravityHarmonic, 3> harmonics{};
};

struct GravityProfile {
  LegGravityProfile left{};
  LegGravityProfile right{};
};

struct SimpleStandingConfig {
  JointVector support_torque_nm{};
  std::array<double, 4> gain{};
  double control_period_s{0.01};
  double control_period_tolerance_s{1.0e-9};
  double maximum_abs_pitch_rad{0.03};
  double maximum_abs_x_m{0.02};
  double maximum_height_error_m{0.01};
  double maximum_leg_error_rad{0.03};
  double maximum_joint_velocity_rad_s{10.0};
};

struct SimpleStanding3dConfig {
  JointVector support_torque_nm{};
  std::array<std::array<double, 8>, 3> gain{};
  JointVector roll_direction{};
  double control_period_s{0.01};
  double control_period_tolerance_s{1.0e-9};
  double maximum_abs_x_m{0.02};
  double maximum_abs_y_m{0.02};
  double maximum_height_error_m{0.01};
  double maximum_abs_roll_rad{0.03};
  double maximum_abs_pitch_rad{0.03};
  double maximum_abs_yaw_rad{0.03};
  double maximum_leg_error_rad{0.03};
  double maximum_joint_velocity_rad_s{10.0};
};

struct WeightedWbcConfig {
  double nominal_height_m{0.0};
  JointVector joint_target_rad{};
  double base_x_kp{0.0};
  double base_x_kd{0.0};
  double height_kp{0.0};
  double height_kd{0.0};
  std::array<double, 3> orientation_kp{};
  std::array<double, 3> orientation_kd{};
  double leg_kp{0.0};
  double leg_kd{0.0};
  Eigen::Matrix<double, 12, 1> interaction_wrench_flu{
      Eigen::Matrix<double, 12, 1>::Zero()};
  double period_s{0.01};
  double period_tolerance_s{1.0e-6};
  double maximum_abs_x_m{0.0};
  double maximum_abs_y_m{0.0};
  double maximum_abs_z_m{0.0};
  double maximum_abs_roll_pitch_rad{0.0};
  double maximum_abs_yaw_rad{0.0};
};

struct NominalNmpcConfig {
  NmpcReferenceProfile reference_profile{NmpcReferenceProfile::kHold};
  double longitudinal_amplitude_m{0.005};
  double step_start_s{0.5};
  double return_start_s{1.5};
  double update_period_s{0.02};
  double deadline_s{0.01};
  NmpcFaultInjection fault_injection{NmpcFaultInjection::kNone};
  std::uint64_t fault_control_tick{100};
};

struct Phase27NmpcConfig {
  double target_common_position_offset_m{0.0};
  double longitudinal_velocity_m_s{0.0};
  double yaw_rate_rad_s{0.0};
  double update_period_s{0.02};
  double deadline_s{0.01};
  NmpcFaultInjection fault_injection{NmpcFaultInjection::kNone};
  std::uint64_t fault_control_tick{100};
};

struct ControllerConfig {
  double quaternion_norm_tolerance{1.0e-6};
  ControllerMode mode{ControllerMode::kZero};
  bool enable_pd{false};
  bool enable_gravity{false};
  JointReference initial_reference{};
  JointVector kp_nm_per_rad{};
  JointVector kd_nm_s_per_rad{};
  JointVector torque_limit_nm{};
  GravityProfile gravity_profile{};
  SimpleStandingConfig simple_standing{};
  SimpleStanding3dConfig simple_standing_3d{};
  WeightedWbcConfig weighted_wbc{};
  NominalNmpcConfig nominal_nmpc{};
  Phase27NmpcConfig phase27_nmpc{};
};

[[nodiscard]] GravityProfile currentNominalGravityProfile();

[[nodiscard]] WeightedWbcConfig currentNominalWeightedWbcConfig();

enum class StepStatus {
  kOk,
  kNotConfigured,
  kInvalidState,
  kNonMonotonicState,
  kSafetyLatched,
};

struct StepResult {
  StepStatus status{StepStatus::kNotConfigured};
  double dt_s{0.0};
  TorqueCommand command{};
  JointVector tau_pd_nm{};
  JointVector tau_gravity_nm{};
  JointVector tau_support_nm{};
  JointVector tau_raw_nm{};
  std::array<bool, kJointCount> saturated{};
  std::array<double, 4> standing_state{};
  std::array<double, 8> standing_state_3d{};
  std::array<double, 3> virtual_input_3d{};
  bool safety_latched{false};
  bool weighted_wbc_active{false};
  bool nominal_nmpc_active{false};
  bool nominal_nmpc_update_tick{false};
  int nominal_nmpc_wrench_age_ticks{0};
  NominalNmpcSolver::Result nominal_nmpc_result{};
  double nominal_nmpc_wbc_total_time_s{0.0};
  WbcReference weighted_wbc_reference{};
  WeightedWbcController::Status weighted_wbc_status{
      WeightedWbcController::Status::kProblemRejected};
  NominalWbcModel::Status weighted_wbc_model_status{
      NominalWbcModel::Status::kInvalidState};
  DenseQpSolver::Status weighted_wbc_solver_status{
      DenseQpSolver::Status::kInvalidInput};
  int weighted_wbc_iterations{0};
  double weighted_wbc_hard_violation{0.0};
  double weighted_wbc_stationarity_residual{0.0};
  double weighted_wbc_primal_residual{0.0};
  double weighted_wbc_dual_residual{0.0};
  NominalWbcModel::Diagnostics weighted_wbc_model_diagnostics{};
  Eigen::Matrix<double, 42, 1> weighted_wbc_physical_solution{
      Eigen::Matrix<double, 42, 1>::Zero()};
  std::array<double, WeightedWbcController::kTaskCount>
      weighted_wbc_task_max_abs_normalized_residual{};
  std::array<double, WeightedWbcController::kTaskCount>
      weighted_wbc_task_normalized_squared_cost{};
  double weighted_wbc_maximum_normalized_slack{0.0};
  Eigen::Matrix<double, 12, 1> weighted_wbc_realized_interaction_wrench_flu{
      Eigen::Matrix<double, 12, 1>::Zero()};
  Eigen::Matrix<double, 12, 1> weighted_wbc_interaction_residual_flu{
      Eigen::Matrix<double, 12, 1>::Zero()};
  Eigen::Matrix<double, 12, 1> weighted_wbc_signed_interaction_slack_flu{
      Eigen::Matrix<double, 12, 1>::Zero()};
  bool phase27_nmpc_active{false};
  bool phase27_nmpc_update_tick{false};
  int phase27_nmpc_wrench_age_ticks{0};
  WheelAwareNmpcSolver::Result phase27_nmpc_result{};
  WheelPositionPlanner::Output phase27_wheel_reference{};
  std::array<double, 2> phase27_wheel_position_b_x_m{};
  std::array<double, 2> phase27_wheel_velocity_b_x_m_s{};
  double phase27_longitudinal_velocity_reference_m_s{0.0};
  double phase27_yaw_rate_reference_rad_s{0.0};
  double phase27_nmpc_wbc_total_time_s{0.0};

  [[nodiscard]] bool accepted() const { return status == StepStatus::kOk; }
};

class ControllerCore {
 public:
  [[nodiscard]] bool configure(const ControllerConfig &config);
  [[nodiscard]] bool setReference(const JointReference &reference);
  [[nodiscard]] bool setPhase27MotionReference(
      double longitudinal_velocity_m_s, double yaw_rate_rad_s);
  void reset();
  [[nodiscard]] StepResult step(const RobotState &state);

 private:
  void stepWeightedWbc(
      const RobotState &state, StepResult &result,
      const NominalNmpcModel::Input *wrench_override = nullptr,
      bool minimal_profile = false);
  void stepNominalNmpcWbc(const RobotState &state, StepResult &result);
  void stepPhase27MinimalNmpcWbc(
      const RobotState &state, StepResult &result);

  ControllerConfig config_{};
  bool configured_{false};
  JointReference reference_{};
  std::optional<std::uint64_t> last_sample_time_ns_;
  std::optional<double> standing_anchor_x_m_;
  std::optional<double> standing_anchor_height_m_;
  std::optional<double> standing_3d_anchor_x_m_;
  std::optional<double> standing_3d_anchor_y_m_;
  std::optional<double> standing_3d_anchor_height_m_;
  std::optional<double> standing_3d_anchor_heading_rad_;
  bool standing_safety_latched_{false};
  WeightedWbcController weighted_wbc_controller_{};
  WeightedWbcController phase27_minimal_wbc_controller_{
      WeightedWbcProfile::kPhase27Minimal};
  std::optional<double> weighted_wbc_anchor_x_m_;
  std::optional<double> weighted_wbc_anchor_y_m_;
  std::optional<double> weighted_wbc_anchor_yaw_rad_;
  std::unique_ptr<NominalNmpcSolver> nominal_nmpc_solver_;
  std::optional<NominalNmpcSolver::Result> nominal_nmpc_last_result_;
  std::uint64_t nominal_nmpc_control_tick_{0};
  std::unique_ptr<WheelAwareNmpcSolver> phase27_nmpc_solver_;
  std::optional<WheelAwareNmpcSolver::Result> phase27_nmpc_last_result_;
  std::uint64_t phase27_nmpc_control_tick_{0};
  NominalWbcModel phase27_state_model_{};
  WheelPositionPlanner phase27_wheel_planner_{};
  std::optional<double> phase27_wheel_target_common_m_;
  std::optional<double> phase27_reference_x_m_;
  std::optional<double> phase27_reference_y_m_;
  std::optional<double> phase27_reference_yaw_rad_;
};

}  // namespace wheel_leg
