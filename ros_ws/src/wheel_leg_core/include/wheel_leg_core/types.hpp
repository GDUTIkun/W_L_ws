#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

namespace wheel_leg {

inline constexpr std::size_t kJointCount = 6;
inline constexpr std::array<std::string_view, kJointCount> kJointNames{
    "left_hip", "left_knee", "left_wheel",
    "right_hip", "right_knee", "right_wheel"};

using Vector3 = std::array<double, 3>;
using QuaternionWxyz = std::array<double, 4>;
using JointVector = std::array<double, kJointCount>;

enum class ContactState : std::uint8_t {
  kUnknown = 0,
  kNoContact = 1,
  kContact = 2,
};

struct RobotState {
  std::uint64_t sample_time_ns{0};
  Vector3 base_position_n_m{};
  QuaternionWxyz q_n_from_b{1.0, 0.0, 0.0, 0.0};
  Vector3 base_linear_velocity_n_m_s{};
  Vector3 base_angular_velocity_n_rad_s{};
  JointVector joint_position_rad{};
  JointVector joint_velocity_rad_s{};
  std::array<ContactState, 2> contact_state{
      ContactState::kUnknown, ContactState::kUnknown};
};

struct TorqueCommand {
  std::uint64_t source_sample_time_ns{0};
  JointVector joint_torque_nm{};
};

enum class ValidationError {
  kNone,
  kNonFinite,
  kQuaternionNorm,
  kInvalidContact,
};

ValidationError validateRobotState(
    const RobotState &state, double quaternion_norm_tolerance = 1.0e-6);
ValidationError validateTorqueCommand(const TorqueCommand &command);

}  // namespace wheel_leg
