#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "wheel_leg_core/types.hpp"

namespace wheel_leg {

enum class ControllerMode {
  kZero,
  kJointPdGravity,
  kSimpleStanding,
  kSimpleStanding3d,
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
};

[[nodiscard]] GravityProfile currentNominalGravityProfile();

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

  [[nodiscard]] bool accepted() const { return status == StepStatus::kOk; }
};

class ControllerCore {
 public:
  [[nodiscard]] bool configure(const ControllerConfig &config);
  [[nodiscard]] bool setReference(const JointReference &reference);
  void reset();
  [[nodiscard]] StepResult step(const RobotState &state);

 private:
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
};

}  // namespace wheel_leg
