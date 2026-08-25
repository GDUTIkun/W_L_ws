#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "wheel_leg_core/types.hpp"

namespace wheel_leg {

enum class ControllerMode {
  kZero,
  kJointPdGravity,
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
};

[[nodiscard]] GravityProfile currentNominalGravityProfile();

enum class StepStatus {
  kOk,
  kNotConfigured,
  kInvalidState,
  kNonMonotonicState,
};

struct StepResult {
  StepStatus status{StepStatus::kNotConfigured};
  double dt_s{0.0};
  TorqueCommand command{};
  JointVector tau_pd_nm{};
  JointVector tau_gravity_nm{};
  JointVector tau_raw_nm{};
  std::array<bool, kJointCount> saturated{};

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
};

}  // namespace wheel_leg
