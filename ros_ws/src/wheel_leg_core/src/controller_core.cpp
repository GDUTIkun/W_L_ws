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

}  // namespace

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
  if (!std::isfinite(config.quaternion_norm_tolerance) ||
      config.quaternion_norm_tolerance < 0.0) {
    return false;
  }
  config_ = config;
  configured_ = true;
  reset();
  return true;
}

void ControllerCore::reset() { last_sample_time_ns_.reset(); }

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
  result.status = StepStatus::kOk;
  return result;
}

}  // namespace wheel_leg
