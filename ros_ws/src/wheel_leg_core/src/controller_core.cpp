#include "wheel_leg_core/controller_core.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace wheel_leg {
namespace {

template <typename Range>
bool allFinite(const Range &values) {
  return std::all_of(
      std::begin(values), std::end(values),
      [](double value) { return std::isfinite(value); });
}

bool validContact(ContactState state) {
  return state == ContactState::kUnknown || state == ContactState::kNoContact ||
         state == ContactState::kContact;
}

bool validReference(const JointReference &reference) {
  return allFinite(reference.position_rad) &&
         allFinite(reference.velocity_rad_s);
}

bool validGravityProfile(const GravityProfile &profile) {
  const auto valid_leg = [](const LegGravityProfile &leg) {
    if (!allFinite(leg.canonical_offset_rad)) {
      return false;
    }
    return std::all_of(
        leg.harmonics.begin(), leg.harmonics.end(),
        [](const GravityHarmonic &term) {
          return std::isfinite(term.sin_torque_nm) &&
                 std::isfinite(term.cos_torque_nm);
        });
  };
  return valid_leg(profile.left) && valid_leg(profile.right);
}

void evaluateLegGravity(
    const LegGravityProfile &profile, const JointVector &position,
    std::size_t first_joint, JointVector &torque) {
  std::array<double, 3> native_position{};
  for (std::size_t joint = 0; joint < native_position.size(); ++joint) {
    native_position[joint] =
        profile.canonical_offset_rad[joint] - position[first_joint + joint];
  }
  for (const auto &term : profile.harmonics) {
    double phase = 0.0;
    for (std::size_t joint = 0; joint < native_position.size(); ++joint) {
      phase += static_cast<double>(term.native_wave_number[joint]) *
               native_position[joint];
    }
    const double scalar = term.sin_torque_nm * std::sin(phase) +
                          term.cos_torque_nm * std::cos(phase);
    for (std::size_t joint = 0; joint < native_position.size(); ++joint) {
      torque[first_joint + joint] +=
          static_cast<double>(term.native_wave_number[joint]) * scalar;
    }
  }
}

}  // namespace

GravityProfile currentNominalGravityProfile() {
  GravityProfile profile;
  profile.left.canonical_offset_rad = {
      -1.3267204090965414, 2.2088002542738268, 0.0};
  profile.right.canonical_offset_rad = {
      -1.3267204090965414, 2.2088002542738268, 0.0};
  profile.left.harmonics = {{
      {{{1, 0, 0}}, -0.45200450502449818, -1.7829178461202144},
      {{{1, 1, 0}}, -1.2121269563787503, 1.4701911157792322},
      {{{1, 1, 1}}, 0.00019620802794421259, 0.00035741999080506598},
  }};
  profile.right.harmonics = {{
      {{{1, 0, 0}}, -0.4519471424166111, -1.7833009239989861},
      {{{1, 1, 0}}, -1.2107526253360692, 1.4713235883642201},
      {{{1, 1, 1}}, -0.00038444708065832856, -0.00012107063092493468},
  }};
  return profile;
}

ValidationError validateRobotState(
    const RobotState &state, double quaternion_norm_tolerance) {
  if (!std::isfinite(quaternion_norm_tolerance) ||
      quaternion_norm_tolerance < 0.0 ||
      !allFinite(state.base_position_n_m) ||
      !allFinite(state.q_n_from_b) ||
      !allFinite(state.base_linear_velocity_n_m_s) ||
      !allFinite(state.base_angular_velocity_n_rad_s) ||
      !allFinite(state.joint_position_rad) ||
      !allFinite(state.joint_velocity_rad_s)) {
    return ValidationError::kNonFinite;
  }

  double norm_squared = 0.0;
  for (const double value : state.q_n_from_b) {
    norm_squared += value * value;
  }
  if (std::abs(std::sqrt(norm_squared) - 1.0) > quaternion_norm_tolerance) {
    return ValidationError::kQuaternionNorm;
  }
  if (!std::all_of(
          state.contact_state.begin(), state.contact_state.end(), validContact)) {
    return ValidationError::kInvalidContact;
  }
  return ValidationError::kNone;
}

ValidationError validateTorqueCommand(const TorqueCommand &command) {
  return allFinite(command.joint_torque_nm) ? ValidationError::kNone
                                            : ValidationError::kNonFinite;
}

bool ControllerCore::configure(const ControllerConfig &config) {
  if ((config.mode != ControllerMode::kZero &&
       config.mode != ControllerMode::kJointPdGravity) ||
      !std::isfinite(config.quaternion_norm_tolerance) ||
      config.quaternion_norm_tolerance < 0.0 ||
      !validReference(config.initial_reference) ||
      !allFinite(config.kp_nm_per_rad) ||
      !allFinite(config.kd_nm_s_per_rad) ||
      !allFinite(config.torque_limit_nm) ||
      !validGravityProfile(config.gravity_profile)) {
    return false;
  }
  if (config.mode == ControllerMode::kJointPdGravity) {
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      if (config.kp_nm_per_rad[joint] < 0.0 ||
          config.kd_nm_s_per_rad[joint] < 0.0 ||
          config.torque_limit_nm[joint] <= 0.0) {
        return false;
      }
    }
  }
  config_ = config;
  configured_ = true;
  reset();
  return true;
}

bool ControllerCore::setReference(const JointReference &reference) {
  if (!configured_ || !validReference(reference)) {
    return false;
  }
  reference_ = reference;
  return true;
}

void ControllerCore::reset() {
  last_sample_time_ns_.reset();
  reference_ = config_.initial_reference;
}

StepResult ControllerCore::step(const RobotState &state) {
  StepResult result;
  result.command.source_sample_time_ns = state.sample_time_ns;
  if (!configured_) {
    return result;
  }
  if (validateRobotState(state, config_.quaternion_norm_tolerance) !=
      ValidationError::kNone) {
    result.status = StepStatus::kInvalidState;
    return result;
  }
  if (last_sample_time_ns_ && state.sample_time_ns <= *last_sample_time_ns_) {
    result.status = StepStatus::kNonMonotonicState;
    return result;
  }
  if (last_sample_time_ns_) {
    result.dt_s = static_cast<double>(
                      state.sample_time_ns - *last_sample_time_ns_) /
                  1.0e9;
  }
  last_sample_time_ns_ = state.sample_time_ns;
  if (config_.mode == ControllerMode::kJointPdGravity) {
    if (config_.enable_pd) {
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        result.tau_pd_nm[joint] =
            config_.kp_nm_per_rad[joint] *
                (reference_.position_rad[joint] -
                 state.joint_position_rad[joint]) +
            config_.kd_nm_s_per_rad[joint] *
                (reference_.velocity_rad_s[joint] -
                 state.joint_velocity_rad_s[joint]);
      }
    }
    if (config_.enable_gravity) {
      evaluateLegGravity(
          config_.gravity_profile.left, state.joint_position_rad, 0,
          result.tau_gravity_nm);
      evaluateLegGravity(
          config_.gravity_profile.right, state.joint_position_rad, 3,
          result.tau_gravity_nm);
    }
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      result.tau_raw_nm[joint] =
          result.tau_pd_nm[joint] + result.tau_gravity_nm[joint];
      const double limit = config_.torque_limit_nm[joint];
      result.command.joint_torque_nm[joint] =
          std::clamp(result.tau_raw_nm[joint], -limit, limit);
      result.saturated[joint] =
          result.command.joint_torque_nm[joint] != result.tau_raw_nm[joint];
    }
  }
  result.status = StepStatus::kOk;
  return result;
}

}  // namespace wheel_leg
