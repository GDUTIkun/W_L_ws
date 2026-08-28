#include "wheel_leg_core/controller_core.hpp"

#include <algorithm>
#include <chrono>
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

bool validSimpleStandingConfig(const SimpleStandingConfig &config) {
  const std::array<double, 7> positive_values{
      config.control_period_s,
      config.control_period_tolerance_s,
      config.maximum_abs_pitch_rad,
      config.maximum_abs_x_m,
      config.maximum_height_error_m,
      config.maximum_leg_error_rad,
      config.maximum_joint_velocity_rad_s,
  };
  return allFinite(config.support_torque_nm) && allFinite(config.gain) &&
         allFinite(positive_values) &&
         std::all_of(
             positive_values.begin(), positive_values.end(),
             [](double value) { return value > 0.0; }) &&
         config.support_torque_nm[2] == 0.0 &&
         config.support_torque_nm[5] == 0.0;
}

bool validSimpleStanding3dConfig(const SimpleStanding3dConfig &config) {
  const std::array<double, 10> positive_values{
      config.control_period_s,
      config.control_period_tolerance_s,
      config.maximum_abs_x_m,
      config.maximum_abs_y_m,
      config.maximum_height_error_m,
      config.maximum_abs_roll_rad,
      config.maximum_abs_pitch_rad,
      config.maximum_abs_yaw_rad,
      config.maximum_leg_error_rad,
      config.maximum_joint_velocity_rad_s,
  };
  double roll_norm_squared = 0.0;
  for (const double value : config.roll_direction) {
    roll_norm_squared += value * value;
  }
  const bool finite_gain = std::all_of(
      config.gain.begin(), config.gain.end(),
      [](const std::array<double, 8> &row) { return allFinite(row); });
  return allFinite(config.support_torque_nm) && finite_gain &&
         allFinite(config.roll_direction) && allFinite(positive_values) &&
         std::all_of(
             positive_values.begin(), positive_values.end(),
             [](double value) { return value > 0.0; }) &&
         config.support_torque_nm[2] == 0.0 &&
         config.support_torque_nm[5] == 0.0 &&
         config.roll_direction[2] == 0.0 &&
         config.roll_direction[5] == 0.0 &&
         std::abs(std::sqrt(roll_norm_squared) - 1.0) <= 1.0e-6;
}

double pitchFromQuaternion(const QuaternionWxyz &quaternion) {
  const auto [w, x, y, z] = quaternion;
  return std::atan2(
      2.0 * (w * y - z * x),
      1.0 - 2.0 * (x * x + y * y));
}

double headingFromQuaternion(const QuaternionWxyz &quaternion) {
  const auto [w, x, y, z] = quaternion;
  return std::atan2(
      2.0 * (w * z + x * y),
      1.0 - 2.0 * (y * y + z * z));
}

QuaternionWxyz multiplyQuaternion(
    const QuaternionWxyz &lhs, const QuaternionWxyz &rhs) {
  const auto [lw, lx, ly, lz] = lhs;
  const auto [rw, rx, ry, rz] = rhs;
  return {
      lw * rw - lx * rx - ly * ry - lz * rz,
      lw * rx + lx * rw + ly * rz - lz * ry,
      lw * ry - lx * rz + ly * rw + lz * rx,
      lw * rz + lx * ry - ly * rx + lz * rw,
  };
}

std::array<double, 3> orientationError(
    const QuaternionWxyz &orientation, double heading_reference_rad) {
  const QuaternionWxyz reference{
      std::cos(heading_reference_rad * 0.5), 0.0, 0.0,
      std::sin(heading_reference_rad * 0.5)};
  auto error = multiplyQuaternion(
      orientation, {reference[0], -reference[1], -reference[2], -reference[3]});
  if (error[0] < 0.0) {
    for (double &value : error) {
      value = -value;
    }
  }
  const double vector_norm =
      std::sqrt(error[1] * error[1] + error[2] * error[2] + error[3] * error[3]);
  if (vector_norm < 1.0e-15) {
    return {};
  }
  const double scale = 2.0 * std::atan2(vector_norm, error[0]) / vector_norm;
  return {scale * error[1], scale * error[2], scale * error[3]};
}

bool isStandingMode(ControllerMode mode) {
  return mode == ControllerMode::kSimpleStanding ||
         mode == ControllerMode::kSimpleStanding3d;
}

bool isSafetyLatchedMode(ControllerMode mode) {
  return isStandingMode(mode) || mode == ControllerMode::kWeightedWbc ||
         mode == ControllerMode::kNominalNmpcWbc;
}

bool validWeightedWbcConfig(
    const WeightedWbcConfig &config, const JointVector &torque_limit_nm) {
  const std::array<double, 7> positive_values{
      config.period_s,
      config.period_tolerance_s,
      config.maximum_abs_x_m,
      config.maximum_abs_y_m,
      config.maximum_abs_z_m,
      config.maximum_abs_roll_pitch_rad,
      config.maximum_abs_yaw_rad,
  };
  const std::array<double, 12> gains{
      config.base_x_kp,       config.base_x_kd,
      config.height_kp,       config.height_kd,
      config.orientation_kp[0], config.orientation_kp[1],
      config.orientation_kp[2], config.orientation_kd[0],
      config.orientation_kd[1], config.orientation_kd[2],
      config.leg_kp,          config.leg_kd,
  };
  return std::isfinite(config.nominal_height_m) &&
         allFinite(config.joint_target_rad) && allFinite(gains) &&
         allFinite(positive_values) &&
         config.interaction_wrench_flu.allFinite() &&
         std::all_of(
             gains.begin(), gains.end(),
             [](double value) { return value > 0.0; }) &&
         std::all_of(
             positive_values.begin(), positive_values.end(),
             [](double value) { return value > 0.0; }) &&
         std::all_of(
             torque_limit_nm.begin(), torque_limit_nm.end(),
             [](double value) { return value > 0.0; });
}

bool validNominalNmpcConfig(const NominalNmpcConfig &config) {
  return std::isfinite(config.longitudinal_amplitude_m) &&
         std::isfinite(config.step_start_s) &&
         std::isfinite(config.return_start_s) &&
         std::isfinite(config.update_period_s) &&
         std::isfinite(config.deadline_s) &&
         config.longitudinal_amplitude_m >= 0.0 &&
         config.longitudinal_amplitude_m <= 0.01 &&
         config.step_start_s >= 0.0 &&
         config.return_start_s > config.step_start_s &&
         std::abs(config.update_period_s - 0.02) <= 1.0e-12 &&
         config.deadline_s > 0.0 && config.deadline_s <= 0.01;
}

bool bothWheelsContact(const RobotState &state) {
  return std::all_of(
      state.contact_state.begin(), state.contact_state.end(),
      [](ContactState contact) { return contact == ContactState::kContact; });
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

WeightedWbcConfig currentNominalWeightedWbcConfig() {
  WeightedWbcConfig config;
  config.nominal_height_m = 0.31543998403249462;
  config.joint_target_rad = {
      -0.97199891583533837, 1.6393957458903228, 0.0,
      -0.98339093564557467, 1.6394010277077622, 0.0};
  config.base_x_kp = 9.0;
  config.base_x_kd = 6.0;
  config.height_kp = 25.0;
  config.height_kd = 10.0;
  config.orientation_kp = {25.0, 25.0, 9.0};
  config.orientation_kd = {10.0, 10.0, 6.0};
  config.leg_kp = 36.0;
  config.leg_kd = 12.0;
  config.interaction_wrench_flu << -0.014600183648230658,
      -0.002144708393769802, 31.572223159792483, 6.939287276081028,
      0.3072127467744123, 2.385752452037761e-05, 0.014600183648233424,
      0.002144708393775349, 31.549240840207528, -6.93345575287898,
      0.39850336198734543, -2.3857524519898827e-05;
  config.period_s = 0.010;
  config.period_tolerance_s = 1.0e-6;
  config.maximum_abs_x_m = 0.02;
  config.maximum_abs_y_m = 0.02;
  config.maximum_abs_z_m = 0.01;
  config.maximum_abs_roll_pitch_rad = 0.03;
  config.maximum_abs_yaw_rad = 0.05;
  return config;
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
       config.mode != ControllerMode::kJointPdGravity &&
       config.mode != ControllerMode::kSimpleStanding &&
       config.mode != ControllerMode::kSimpleStanding3d &&
       config.mode != ControllerMode::kWeightedWbc &&
       config.mode != ControllerMode::kNominalNmpcWbc) ||
      !std::isfinite(config.quaternion_norm_tolerance) ||
      config.quaternion_norm_tolerance < 0.0 ||
      !validReference(config.initial_reference) ||
      !allFinite(config.kp_nm_per_rad) ||
      !allFinite(config.kd_nm_s_per_rad) ||
      !allFinite(config.torque_limit_nm) ||
      !validGravityProfile(config.gravity_profile) ||
      (config.mode == ControllerMode::kSimpleStanding &&
       !validSimpleStandingConfig(config.simple_standing)) ||
      (config.mode == ControllerMode::kSimpleStanding3d &&
       !validSimpleStanding3dConfig(config.simple_standing_3d)) ||
      ((config.mode == ControllerMode::kWeightedWbc ||
        config.mode == ControllerMode::kNominalNmpcWbc) &&
       !validWeightedWbcConfig(config.weighted_wbc, config.torque_limit_nm)) ||
      (config.mode == ControllerMode::kNominalNmpcWbc &&
       !validNominalNmpcConfig(config.nominal_nmpc))) {
    return false;
  }
  if (config.mode == ControllerMode::kJointPdGravity ||
      isStandingMode(config.mode)) {
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      if (config.kp_nm_per_rad[joint] < 0.0 ||
          config.kd_nm_s_per_rad[joint] < 0.0 ||
          config.torque_limit_nm[joint] <= 0.0) {
        return false;
      }
    }
  }
  config_ = config;
  if (config.mode == ControllerMode::kNominalNmpcWbc) {
    nominal_nmpc_solver_ = std::make_unique<NominalNmpcSolver>();
    if (!nominal_nmpc_solver_->ready()) {
      nominal_nmpc_solver_.reset();
      return false;
    }
  } else {
    nominal_nmpc_solver_.reset();
  }
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
  standing_anchor_x_m_.reset();
  standing_anchor_height_m_.reset();
  standing_3d_anchor_x_m_.reset();
  standing_3d_anchor_y_m_.reset();
  standing_3d_anchor_height_m_.reset();
  standing_3d_anchor_heading_rad_.reset();
  standing_safety_latched_ = false;
  weighted_wbc_controller_.reset();
  weighted_wbc_anchor_x_m_.reset();
  weighted_wbc_anchor_y_m_.reset();
  weighted_wbc_anchor_yaw_rad_.reset();
  nominal_nmpc_last_result_.reset();
  nominal_nmpc_control_tick_ = 0;
  if (nominal_nmpc_solver_) nominal_nmpc_solver_->reset();
}

void ControllerCore::stepWeightedWbc(
    const RobotState &state, StepResult &result,
    const NominalNmpcModel::Input *wrench_override) {
  const auto &wbc = config_.weighted_wbc;
  if (result.dt_s != 0.0 &&
      std::abs(result.dt_s - wbc.period_s) > wbc.period_tolerance_s) {
    standing_safety_latched_ = true;
  }
  if (!weighted_wbc_anchor_x_m_) {
    weighted_wbc_anchor_x_m_ = state.base_position_n_m[0];
    weighted_wbc_anchor_y_m_ = state.base_position_n_m[1];
    weighted_wbc_anchor_yaw_rad_ = headingFromQuaternion(state.q_n_from_b);
  }
  const auto orientation = orientationError(
      state.q_n_from_b, *weighted_wbc_anchor_yaw_rad_);
  const bool safety_violated = standing_safety_latched_ ||
      !bothWheelsContact(state) ||
      std::abs(state.base_position_n_m[0] - *weighted_wbc_anchor_x_m_) >
          wbc.maximum_abs_x_m ||
      std::abs(state.base_position_n_m[1] - *weighted_wbc_anchor_y_m_) >
          wbc.maximum_abs_y_m ||
      std::abs(state.base_position_n_m[2] - wbc.nominal_height_m) >
          wbc.maximum_abs_z_m ||
      std::abs(orientation[0]) > wbc.maximum_abs_roll_pitch_rad ||
      std::abs(orientation[1]) > wbc.maximum_abs_roll_pitch_rad ||
      std::abs(orientation[2]) > wbc.maximum_abs_yaw_rad;
  WbcReference reference;
  const std::array<std::size_t, 4> leg_joints{0, 1, 3, 4};
  reference.base_x_acceleration_m_s2 =
      -wbc.base_x_kp *
          (state.base_position_n_m[0] - *weighted_wbc_anchor_x_m_) -
      wbc.base_x_kd * state.base_linear_velocity_n_m_s[0];
  reference.base_height_acceleration_m_s2 =
      -wbc.height_kp * (state.base_position_n_m[2] - wbc.nominal_height_m) -
      wbc.height_kd * state.base_linear_velocity_n_m_s[2];
  for (std::size_t axis = 0; axis < 3; ++axis) {
    reference.orientation_acceleration_rad_s2[static_cast<Eigen::Index>(axis)] =
        -wbc.orientation_kp[axis] * orientation[axis] -
        wbc.orientation_kd[axis] * state.base_angular_velocity_n_rad_s[axis];
  }
  for (std::size_t index = 0; index < leg_joints.size(); ++index) {
    const std::size_t joint = leg_joints[index];
    reference.leg_acceleration_rad_s2[static_cast<Eigen::Index>(index)] =
        wbc.leg_kp *
            (wbc.joint_target_rad[joint] - state.joint_position_rad[joint]) -
        wbc.leg_kd * state.joint_velocity_rad_s[joint];
  }
  reference.interaction_wrench_flu =
      wrench_override == nullptr ? wbc.interaction_wrench_flu : *wrench_override;
  result.weighted_wbc_reference = reference;
  if (safety_violated) {
    standing_safety_latched_ = true;
    result.status = StepStatus::kSafetyLatched;
    result.safety_latched = true;
    return;
  }
  const auto wbc_result = weighted_wbc_controller_.step(state, reference);
  result.weighted_wbc_status = wbc_result.status;
  result.weighted_wbc_model_status = wbc_result.model_status;
  result.weighted_wbc_solver_status = wbc_result.solver_status;
  result.weighted_wbc_iterations = wbc_result.iterations;
  result.weighted_wbc_hard_violation = wbc_result.hard_violation;
  result.weighted_wbc_stationarity_residual =
      wbc_result.stationarity_residual;
  result.weighted_wbc_primal_residual = wbc_result.primal_residual;
  result.weighted_wbc_dual_residual = wbc_result.dual_residual;
  result.weighted_wbc_model_diagnostics = wbc_result.model_diagnostics;
  result.weighted_wbc_physical_solution = wbc_result.physical_solution;
  result.weighted_wbc_task_max_abs_normalized_residual =
      wbc_result.task_max_abs_normalized_residual;
  result.weighted_wbc_task_normalized_squared_cost =
      wbc_result.task_normalized_squared_cost;
  result.weighted_wbc_maximum_normalized_slack =
      wbc_result.maximum_normalized_slack;
  bool over_limit = false;
  for (std::size_t joint = 0; joint < kJointCount; ++joint) {
    if (std::abs(wbc_result.torque_nm[joint]) >
        config_.torque_limit_nm[joint]) {
      result.saturated[joint] = true;
      over_limit = true;
    }
  }
  if (!wbc_result.ok() || !allFinite(wbc_result.torque_nm) || over_limit) {
    standing_safety_latched_ = true;
    result.status = StepStatus::kSafetyLatched;
    result.safety_latched = true;
    return;
  }
  result.tau_raw_nm = wbc_result.torque_nm;
  result.command.joint_torque_nm = wbc_result.torque_nm;
  result.status = StepStatus::kOk;
}

void ControllerCore::stepNominalNmpcWbc(
    const RobotState &state, StepResult &result) {
  using Clock = std::chrono::steady_clock;
  const auto start = Clock::now();
  if (!weighted_wbc_anchor_x_m_) {
    weighted_wbc_anchor_x_m_ = state.base_position_n_m[0];
    weighted_wbc_anchor_y_m_ = state.base_position_n_m[1];
    weighted_wbc_anchor_yaw_rad_ = headingFromQuaternion(state.q_n_from_b);
  }
  const bool update_tick = nominal_nmpc_control_tick_ % 2 == 0;
  result.nominal_nmpc_update_tick = update_tick;
  if (update_tick) {
    NominalNmpcProblem problem;
    for (std::size_t axis = 0; axis < 3; ++axis) {
      problem.state[static_cast<Eigen::Index>(axis)] =
          state.base_position_n_m[axis];
      problem.state[static_cast<Eigen::Index>(6 + axis)] =
          state.base_linear_velocity_n_m_s[axis];
      problem.state[static_cast<Eigen::Index>(9 + axis)] =
          state.base_angular_velocity_n_rad_s[axis];
    }
    const auto rotation = orientationError(
        state.q_n_from_b, *weighted_wbc_anchor_yaw_rad_);
    for (std::size_t axis = 0; axis < 3; ++axis) {
      problem.state[static_cast<Eigen::Index>(3 + axis)] = rotation[axis];
    }
    problem.reference = problem.state;
    problem.reference.segment<3>(6).setZero();
    problem.reference.segment<3>(9).setZero();
    problem.reference.segment<3>(3).setZero();
    problem.reference[0] = *weighted_wbc_anchor_x_m_;
    problem.reference[1] = *weighted_wbc_anchor_y_m_;
    problem.reference[2] = config_.weighted_wbc.nominal_height_m;
    const double elapsed_s =
        static_cast<double>(nominal_nmpc_control_tick_) *
        config_.weighted_wbc.period_s;
    if (elapsed_s >= config_.nominal_nmpc.step_start_s) {
      if (config_.nominal_nmpc.reference_profile ==
          NmpcReferenceProfile::kPositiveStep) {
        problem.reference[0] += config_.nominal_nmpc.longitudinal_amplitude_m;
      } else if (config_.nominal_nmpc.reference_profile ==
                 NmpcReferenceProfile::kNegativeStep) {
        problem.reference[0] -= config_.nominal_nmpc.longitudinal_amplitude_m;
      } else if (config_.nominal_nmpc.reference_profile ==
                     NmpcReferenceProfile::kStepReturn &&
                 elapsed_s < config_.nominal_nmpc.return_start_s) {
        problem.reference[0] += config_.nominal_nmpc.longitudinal_amplitude_m;
      }
    }
    const double yaw = *weighted_wbc_anchor_yaw_rad_;
    problem.reference_rotation_n_from_b <<
        std::cos(yaw), -std::sin(yaw), 0.0,
        std::sin(yaw), std::cos(yaw), 0.0,
        0.0, 0.0, 1.0;
    result.nominal_nmpc_result = nominal_nmpc_solver_->solve(problem);
    nominal_nmpc_last_result_ = result.nominal_nmpc_result;
    result.nominal_nmpc_wrench_age_ticks = 0;
  } else if (nominal_nmpc_last_result_) {
    result.nominal_nmpc_result = *nominal_nmpc_last_result_;
    result.nominal_nmpc_wrench_age_ticks = 1;
  }
  ++nominal_nmpc_control_tick_;
  if (!nominal_nmpc_last_result_ || !nominal_nmpc_last_result_->ok() ||
      result.nominal_nmpc_wrench_age_ticks > 1) {
    standing_safety_latched_ = true;
    result.status = StepStatus::kSafetyLatched;
    result.safety_latched = true;
    return;
  }
  stepWeightedWbc(
      state, result, &nominal_nmpc_last_result_->wrench_flu);
  result.nominal_nmpc_wbc_total_time_s =
      std::chrono::duration<double>(Clock::now() - start).count();
  if (result.nominal_nmpc_wbc_total_time_s >
      config_.nominal_nmpc.deadline_s) {
    result.command.joint_torque_nm.fill(0.0);
    result.tau_raw_nm.fill(0.0);
    standing_safety_latched_ = true;
    result.status = StepStatus::kSafetyLatched;
    result.safety_latched = true;
  }
}

StepResult ControllerCore::step(const RobotState &state) {
  StepResult result;
  result.command.source_sample_time_ns = state.sample_time_ns;
  if (!configured_) {
    return result;
  }
  const bool nmpc_mode = config_.mode == ControllerMode::kNominalNmpcWbc;
  const bool weighted_mode =
      config_.mode == ControllerMode::kWeightedWbc || nmpc_mode;
  if (weighted_mode) {
    result.weighted_wbc_active = true;
  }
  result.nominal_nmpc_active = nmpc_mode;
  if (isSafetyLatchedMode(config_.mode) && standing_safety_latched_) {
    result.status = StepStatus::kSafetyLatched;
    result.safety_latched = true;
    return result;
  }
  if (validateRobotState(state, config_.quaternion_norm_tolerance) !=
      ValidationError::kNone) {
    standing_safety_latched_ = isSafetyLatchedMode(config_.mode);
    result.status = StepStatus::kInvalidState;
    result.safety_latched = standing_safety_latched_;
    return result;
  }
  if (last_sample_time_ns_ && state.sample_time_ns <= *last_sample_time_ns_) {
    standing_safety_latched_ = isSafetyLatchedMode(config_.mode);
    result.status = StepStatus::kNonMonotonicState;
    result.safety_latched = standing_safety_latched_;
    return result;
  }
  if (last_sample_time_ns_) {
    result.dt_s = static_cast<double>(
                      state.sample_time_ns - *last_sample_time_ns_) /
                  1.0e9;
  }
  last_sample_time_ns_ = state.sample_time_ns;
  if (nmpc_mode) {
    stepNominalNmpcWbc(state, result);
    return result;
  }
  if (weighted_mode) {
    stepWeightedWbc(state, result);
    return result;
  }
  if (config_.mode == ControllerMode::kSimpleStanding3d) {
    const auto &standing = config_.simple_standing_3d;
    if (result.dt_s != 0.0 &&
        std::abs(result.dt_s - standing.control_period_s) >
            standing.control_period_tolerance_s) {
      standing_safety_latched_ = true;
    }
    if (!standing_3d_anchor_x_m_) {
      standing_3d_anchor_x_m_ = state.base_position_n_m[0];
      standing_3d_anchor_y_m_ = state.base_position_n_m[1];
      standing_3d_anchor_height_m_ = state.base_position_n_m[2];
      standing_3d_anchor_heading_rad_ = headingFromQuaternion(state.q_n_from_b);
    }
    const auto orientation = orientationError(
        state.q_n_from_b, *standing_3d_anchor_heading_rad_);
    result.standing_state_3d = {
        state.base_position_n_m[0] - *standing_3d_anchor_x_m_,
        state.base_linear_velocity_n_m_s[0],
        orientation[1], state.base_angular_velocity_n_rad_s[1],
        orientation[0], state.base_angular_velocity_n_rad_s[0],
        orientation[2], state.base_angular_velocity_n_rad_s[2],
    };
    const std::array<std::size_t, 4> leg_joints{0, 1, 3, 4};
    const bool leg_error_exceeded = std::any_of(
        leg_joints.begin(), leg_joints.end(), [&](std::size_t joint) {
          return std::abs(
                     state.joint_position_rad[joint] -
                     reference_.position_rad[joint]) >
                 standing.maximum_leg_error_rad;
        });
    standing_safety_latched_ = standing_safety_latched_ ||
        !bothWheelsContact(state) ||
        std::abs(result.standing_state_3d[0]) > standing.maximum_abs_x_m ||
        std::abs(state.base_position_n_m[1] - *standing_3d_anchor_y_m_) >
            standing.maximum_abs_y_m ||
        std::abs(state.base_position_n_m[2] - *standing_3d_anchor_height_m_) >
            standing.maximum_height_error_m ||
        std::abs(result.standing_state_3d[2]) > standing.maximum_abs_pitch_rad ||
        std::abs(result.standing_state_3d[4]) > standing.maximum_abs_roll_rad ||
        std::abs(result.standing_state_3d[6]) > standing.maximum_abs_yaw_rad ||
        leg_error_exceeded || std::any_of(
            state.joint_velocity_rad_s.begin(), state.joint_velocity_rad_s.end(),
            [&](double velocity) {
              return std::abs(velocity) > standing.maximum_joint_velocity_rad_s;
            });
    if (!standing_safety_latched_) {
      for (std::size_t input = 0; input < result.virtual_input_3d.size(); ++input) {
        for (std::size_t index = 0; index < result.standing_state_3d.size(); ++index) {
          result.virtual_input_3d[input] -=
              standing.gain[input][index] * result.standing_state_3d[index];
        }
      }
      for (const std::size_t joint : leg_joints) {
        result.tau_pd_nm[joint] =
            config_.kp_nm_per_rad[joint] *
                (reference_.position_rad[joint] - state.joint_position_rad[joint]) +
            config_.kd_nm_s_per_rad[joint] *
                (reference_.velocity_rad_s[joint] - state.joint_velocity_rad_s[joint]);
        result.tau_support_nm[joint] = standing.support_torque_nm[joint];
        result.tau_raw_nm[joint] = result.tau_support_nm[joint] + result.tau_pd_nm[joint] +
            standing.roll_direction[joint] * result.virtual_input_3d[1];
      }
      result.tau_raw_nm[2] = result.virtual_input_3d[0] + result.virtual_input_3d[2];
      result.tau_raw_nm[5] = result.virtual_input_3d[0] - result.virtual_input_3d[2];
      if (!allFinite(result.tau_raw_nm)) {
        standing_safety_latched_ = true;
      }
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        if (std::abs(result.tau_raw_nm[joint]) > config_.torque_limit_nm[joint]) {
          result.saturated[joint] = true;
          standing_safety_latched_ = true;
        }
      }
    }
    if (standing_safety_latched_) {
      result.status = StepStatus::kSafetyLatched;
      result.safety_latched = true;
      return result;
    }
    result.command.joint_torque_nm = result.tau_raw_nm;
    result.status = StepStatus::kOk;
    return result;
  }
  if (config_.mode == ControllerMode::kSimpleStanding) {
    const auto &standing = config_.simple_standing;
    if (result.dt_s != 0.0 &&
        std::abs(result.dt_s - standing.control_period_s) >
            standing.control_period_tolerance_s) {
      standing_safety_latched_ = true;
    }
    if (!standing_anchor_x_m_) {
      standing_anchor_x_m_ = state.base_position_n_m[0];
      standing_anchor_height_m_ = state.base_position_n_m[2];
    }
    result.standing_state = {
        state.base_position_n_m[0] - *standing_anchor_x_m_,
        state.base_linear_velocity_n_m_s[0],
        pitchFromQuaternion(state.q_n_from_b),
        state.base_angular_velocity_n_rad_s[1],
    };
    const std::array<std::size_t, 4> leg_joints{0, 1, 3, 4};
    const bool leg_error_exceeded = std::any_of(
        leg_joints.begin(), leg_joints.end(), [&](std::size_t joint) {
          return std::abs(
                     state.joint_position_rad[joint] -
                     reference_.position_rad[joint]) >
                 standing.maximum_leg_error_rad;
        });
    standing_safety_latched_ = standing_safety_latched_ ||
        !bothWheelsContact(state) ||
        std::abs(result.standing_state[0]) > standing.maximum_abs_x_m ||
        std::abs(result.standing_state[2]) > standing.maximum_abs_pitch_rad ||
        std::abs(state.base_position_n_m[2] - *standing_anchor_height_m_) >
            standing.maximum_height_error_m ||
        leg_error_exceeded ||
        std::any_of(
            state.joint_velocity_rad_s.begin(),
            state.joint_velocity_rad_s.end(), [&](double velocity) {
              return std::abs(velocity) >
                     standing.maximum_joint_velocity_rad_s;
            });
    if (!standing_safety_latched_) {
      for (const std::size_t joint : leg_joints) {
        result.tau_pd_nm[joint] =
            config_.kp_nm_per_rad[joint] *
                (reference_.position_rad[joint] -
                 state.joint_position_rad[joint]) +
            config_.kd_nm_s_per_rad[joint] *
                (reference_.velocity_rad_s[joint] -
                 state.joint_velocity_rad_s[joint]);
        result.tau_support_nm[joint] = standing.support_torque_nm[joint];
      }
      double wheel_torque = 0.0;
      for (std::size_t index = 0; index < result.standing_state.size(); ++index) {
        wheel_torque -= standing.gain[index] * result.standing_state[index];
      }
      result.tau_raw_nm[2] = wheel_torque;
      result.tau_raw_nm[5] = wheel_torque;
      for (const std::size_t joint : leg_joints) {
        result.tau_raw_nm[joint] =
            result.tau_support_nm[joint] + result.tau_pd_nm[joint];
      }
      for (std::size_t joint = 0; joint < kJointCount; ++joint) {
        if (std::abs(result.tau_raw_nm[joint]) >
            config_.torque_limit_nm[joint]) {
          result.saturated[joint] = true;
          standing_safety_latched_ = true;
        }
      }
    }
    if (standing_safety_latched_) {
      result.status = StepStatus::kSafetyLatched;
      result.safety_latched = true;
      return result;
    }
    result.command.joint_torque_nm = result.tau_raw_nm;
    result.status = StepStatus::kOk;
    return result;
  }
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
