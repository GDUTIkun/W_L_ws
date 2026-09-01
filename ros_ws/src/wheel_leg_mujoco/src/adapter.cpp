#include "wheel_leg_mujoco/adapter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "wheel_leg_core/controller_core.hpp"

namespace wheel_leg_mujoco {
namespace {

constexpr std::array<const char *, wheel_leg::kJointCount> kJointNames{
    "left_hip_joint", "left_knee_joint", "left_wheel_joint",
    "right_hip_joint", "right_knee_joint", "right_wheel_joint"};
constexpr std::array<const char *, wheel_leg::kJointCount> kActuatorNames{
    "left_hip_torque", "left_knee_torque", "left_wheel_torque",
    "right_hip_torque", "right_knee_torque", "right_wheel_torque"};
constexpr std::array<const char *, 2> kWheelGeomNames{
    "left_wheel_collision", "right_wheel_collision"};

int requiredId(const mjModel *model, mjtObj type, const char *name) {
  const int id = mj_name2id(model, type, name);
  if (id < 0) {
    throw std::invalid_argument(std::string("Missing MuJoCo object: ") + name);
  }
  return id;
}

bool pairMatches(int geom1, int geom2, int first, int second) {
  return (geom1 == first && geom2 == second) ||
         (geom1 == second && geom2 == first);
}

}  // namespace

void initializeCurrentWeightedWbcH0(const mjModel *model, mjData *data) {
  if (model == nullptr || data == nullptr || model->nq != 17 || model->nv != 16) {
    throw std::invalid_argument("Invalid MuJoCo model/data for current WBC H0");
  }
  constexpr std::array<double, 9> equilibrium{
      -0.34332947374181766, 0.5693992271789607,
      -0.35472149355205396, 0.5694045089964002,
      0.34771766403249466, -0.572545089643551,
      -0.5729875480645877, 0.5725979309569537,
      -0.5730345859812999};
  constexpr std::array<const char *, 4> active{
      "right_hip_joint", "right_knee_joint", "left_hip_joint", "left_knee_joint"};
  constexpr std::array<const char *, 4> passive{
      "right_connect1_joint", "right_connect2_joint",
      "left_connect1_joint", "left_connect2_joint"};

  data->qpos[2] = equilibrium[4];
  data->qpos[3] = 1.0;
  for (std::size_t index = 0; index < active.size(); ++index) {
    data->qpos[model->jnt_qposadr[requiredId(model, mjOBJ_JOINT, active[index])]] =
        equilibrium[index];
    data->qpos[model->jnt_qposadr[requiredId(model, mjOBJ_JOINT, passive[index])]] =
        equilibrium[index + 5];
  }
  mj_forward(model, data);
}

Adapter::Adapter(const mjModel *model, AdapterConfig config)
    : model_(model), config_(config) {
  if (std::string(mj_versionString()) != "3.7.0") {
    throw std::invalid_argument("wheel_leg_mujoco requires MuJoCo 3.7.0");
  }
  if (model_ == nullptr ||
      model_->nu != static_cast<int>(wheel_leg::kJointCount) ||
      !std::isfinite(model_->opt.timestep) ||
      std::abs(model_->opt.timestep - 0.002) > 1.0e-12 ||
      config_.command_timeout_ns == 0 || config_.max_source_lag_ns == 0 ||
      !std::all_of(
          config_.joint_offsets_rad.begin(), config_.joint_offsets_rad.end(),
          [](double value) { return std::isfinite(value); })) {
    throw std::invalid_argument("Invalid MuJoCo Adapter model or configuration");
  }

  base_control_site_id_ =
      requiredId(model_, mjOBJ_SITE, "base_control_frame");
  floor_geom_id_ = requiredId(model_, mjOBJ_GEOM, "floor");
  base_weld_id_ = requiredId(model_, mjOBJ_EQUALITY, "base_weld");
  for (std::size_t index = 0; index < wheel_leg::kJointCount; ++index) {
    joint_ids_[index] = requiredId(model_, mjOBJ_JOINT, kJointNames[index]);
    actuator_ids_[index] =
        requiredId(model_, mjOBJ_ACTUATOR, kActuatorNames[index]);
    const int actuator_id = actuator_ids_[index];
    if (model_->actuator_trnid[2 * actuator_id] != joint_ids_[index] ||
        std::abs(model_->actuator_gear[6 * actuator_id] - 1.0) > 1.0e-12) {
      throw std::invalid_argument("Actuator joint/order/gear invariant failed");
    }
  }
  for (std::size_t index = 0; index < wheel_geom_ids_.size(); ++index) {
    wheel_geom_ids_[index] =
        requiredId(model_, mjOBJ_GEOM, kWheelGeomNames[index]);
  }
}

std::uint64_t Adapter::simulationTimeNs(double time_s) {
  if (!std::isfinite(time_s) || time_s < 0.0 ||
      time_s > static_cast<double>(std::numeric_limits<std::uint64_t>::max()) /
                   1.0e9) {
    throw std::invalid_argument("Invalid MuJoCo simulation time");
  }
  return static_cast<std::uint64_t>(std::llround(time_s * 1.0e9));
}

void Adapter::reset(mjData *data) {
  if (data == nullptr) {
    throw std::invalid_argument("Cannot reset null mjData");
  }
  mj_resetData(model_, data);
  data->eq_active[base_weld_id_] = config_.floating_base ? 0 : 1;
  std::fill(data->ctrl, data->ctrl + model_->nu, 0.0);
  command_.reset();
  command_receipt_time_ns_.reset();
  last_command_source_time_ns_.reset();
  last_quaternion_.reset();
  mj_forward(model_, data);
}

wheel_leg::RobotState Adapter::extractState(mjData *data) {
  if (data == nullptr) {
    throw std::invalid_argument("Cannot extract state from null mjData");
  }
  mj_forward(model_, data);

  wheel_leg::RobotState state;
  state.sample_time_ns = simulationTimeNs(data->time);
  const mjtNum *site_position = data->site_xpos + 3 * base_control_site_id_;
  std::copy_n(site_position, 3, state.base_position_n_m.begin());

  mju_mat2Quat(
      state.q_n_from_b.data(), data->site_xmat + 9 * base_control_site_id_);
  if (last_quaternion_) {
    double dot = 0.0;
    for (std::size_t index = 0; index < state.q_n_from_b.size(); ++index) {
      dot += state.q_n_from_b[index] * (*last_quaternion_)[index];
    }
    if (dot < 0.0) {
      for (double &value : state.q_n_from_b) {
        value = -value;
      }
    }
  }
  last_quaternion_ = state.q_n_from_b;

  std::vector<mjtNum> jacobian_position(3 * model_->nv);
  std::vector<mjtNum> jacobian_rotation(3 * model_->nv);
  mj_jacSite(
      model_, data, jacobian_position.data(), jacobian_rotation.data(),
      base_control_site_id_);
  mju_mulMatVec(
      state.base_linear_velocity_n_m_s.data(), jacobian_position.data(),
      data->qvel, 3, model_->nv);
  mju_mulMatVec(
      state.base_angular_velocity_n_rad_s.data(), jacobian_rotation.data(),
      data->qvel, 3, model_->nv);

  for (std::size_t index = 0; index < wheel_leg::kJointCount; ++index) {
    const int joint_id = joint_ids_[index];
    state.joint_position_rad[index] =
        -data->qpos[model_->jnt_qposadr[joint_id]] +
        config_.joint_offsets_rad[index];
    state.joint_velocity_rad_s[index] =
        -data->qvel[model_->jnt_dofadr[joint_id]];
  }

  state.contact_state = {
      wheel_leg::ContactState::kNoContact,
      wheel_leg::ContactState::kNoContact};
  for (int contact_index = 0; contact_index < data->ncon; ++contact_index) {
    const mjContact &contact = data->contact[contact_index];
    for (std::size_t side = 0; side < wheel_geom_ids_.size(); ++side) {
      if (pairMatches(
              contact.geom1, contact.geom2, wheel_geom_ids_[side],
              floor_geom_id_)) {
        state.contact_state[side] = wheel_leg::ContactState::kContact;
      }
    }
  }

  if (wheel_leg::validateRobotState(state) != wheel_leg::ValidationError::kNone) {
    throw std::runtime_error("MuJoCo produced an invalid canonical RobotState");
  }
  return state;
}

bool Adapter::acceptCommand(
    const wheel_leg::TorqueCommand &command,
    std::uint64_t receipt_time_ns,
    std::uint64_t current_source_time_ns) {
  if (wheel_leg::validateTorqueCommand(command) !=
          wheel_leg::ValidationError::kNone ||
      command.source_sample_time_ns > current_source_time_ns ||
      current_source_time_ns - command.source_sample_time_ns >
          config_.max_source_lag_ns ||
      (last_command_source_time_ns_ &&
       command.source_sample_time_ns <= *last_command_source_time_ns_)) {
    command_.reset();
    command_receipt_time_ns_.reset();
    return false;
  }
  command_ = command;
  command_receipt_time_ns_ = receipt_time_ns;
  last_command_source_time_ns_ = command.source_sample_time_ns;
  return true;
}

void Adapter::writeControls(mjData *data, std::uint64_t receipt_time_ns) {
  if (data == nullptr) {
    throw std::invalid_argument("Cannot write controls to null mjData");
  }
  std::fill(data->ctrl, data->ctrl + model_->nu, 0.0);
  if (!config_.command_enabled || !command_ || !command_receipt_time_ns_ ||
      receipt_time_ns < *command_receipt_time_ns_ ||
      receipt_time_ns - *command_receipt_time_ns_ > config_.command_timeout_ns) {
    return;
  }
  const std::uint64_t source_now = simulationTimeNs(data->time);
  if (command_->source_sample_time_ns > source_now ||
      source_now - command_->source_sample_time_ns >
          config_.max_source_lag_ns) {
    return;
  }
  for (std::size_t index = 0; index < wheel_leg::kJointCount; ++index) {
    data->ctrl[actuator_ids_[index]] = -command_->joint_torque_nm[index];
  }
}

}  // namespace wheel_leg_mujoco
