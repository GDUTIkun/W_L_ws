#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

#include "mujoco/mujoco.h"
#include "wheel_leg_core/weighted_wbc_controller.hpp"
#include "wheel_leg_core/wheel_position_planner.hpp"
#include "wheel_leg_mujoco/adapter.hpp"

namespace {

struct ModelDeleter { void operator()(mjModel *value) const { mj_deleteModel(value); } };
struct DataDeleter { void operator()(mjData *value) const { mj_deleteData(value); } };
using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;

constexpr std::array<const char *, 10> kRawJointNames{
    "left_hip_joint", "left_knee_joint", "left_wheel_joint",
    "right_hip_joint", "right_knee_joint", "right_wheel_joint",
    "left_connect1_joint", "left_connect2_joint",
    "right_connect1_joint", "right_connect2_joint"};
constexpr std::array<const char *, 2> kWheelGeomNames{
    "left_wheel_collision", "right_wheel_collision"};
constexpr std::array<double, 6> kTorqueLimit{10.0, 10.0, 2.0, 10.0, 10.0, 2.0};

int requiredId(const mjModel *model, mjtObj type, const char *name) {
  const int id = mj_name2id(model, type, name);
  if (id < 0) throw std::runtime_error(std::string("missing MuJoCo object: ") + name);
  return id;
}

void setInitialState(const mjModel *model, mjData *data) {
  constexpr std::array<double, 9> equilibrium{
      -0.34332947374181766, 0.5693992271789607,
      -0.35472149355205396, 0.5694045089964002,
      0.34771766403249466, -0.572545089643551,
      -0.5729875480645877, 0.5725979309569537,
      -0.5730345859812999};
  data->qpos[2] = equilibrium[4];
  data->qpos[3] = 1.0;
  constexpr std::array<const char *, 4> active{
      "right_hip_joint", "right_knee_joint", "left_hip_joint", "left_knee_joint"};
  constexpr std::array<const char *, 4> passive{
      "right_connect1_joint", "right_connect2_joint",
      "left_connect1_joint", "left_connect2_joint"};
  for (std::size_t index = 0; index < active.size(); ++index) {
    data->qpos[model->jnt_qposadr[requiredId(model, mjOBJ_JOINT, active[index])]] =
        equilibrium[index];
    data->qpos[model->jnt_qposadr[requiredId(model, mjOBJ_JOINT, passive[index])]] =
        equilibrium[index + 5];
  }
  mj_forward(model, data);
  const int site = requiredId(model, mjOBJ_SITE, "base_control_frame");
  const double equilibrium_site_x = data->site_xpos[3 * site];
  data->qpos[0] += equilibrium_site_x - data->site_xpos[3 * site];
  mj_forward(model, data);
}

wheel_leg::WbcReference equilibriumReference() {
  wheel_leg::WbcReference reference;
  reference.interaction_wrench_flu <<
      0.0, 0.0, 27.675229491866027, 0.11327183296816838, 0.0, 0.0,
      0.0, 0.0, 28.714612508133982, 0.11327183296816838, 0.0, 0.0;
  return reference;
}

bool pairMatches(int geom1, int geom2, int first, int second) {
  return (geom1 == first && geom2 == second) ||
         (geom1 == second && geom2 == first);
}

void writeHeader(std::ostream &output) {
  output << "tick,time_s,case_id,gain_id,activation,model_status,controller_status"
         << ",solver_status,iterations,wbc_time_s,hard,primal,dual,stationarity"
         << ",contact_left,contact_right,normal_left,normal_right"
         << ",planner_xi,planner_dxi,planner_ddxi,desired_ddxi_left,desired_ddxi_right"
         << ",xi_left,xi_right,dxi_left,dxi_right,xi_common,xi_differential"
         << ",dxi_common,dxi_differential,zeta_left,zeta_right,dzeta_left,dzeta_right";
  for (int side = 0; side < 2; ++side)
    for (int axis = 0; axis < 3; ++axis)
      output << ",raw_wheel_p" << side << '_' << axis
             << ",raw_wheel_v" << side << '_' << axis;
  output << ",physical_ddxi_left,physical_ddxi_right";
  for (int joint = 0; joint < 6; ++joint) {
    output << ",q" << joint << ",dq" << joint << ",qeq" << joint
           << ",delta" << joint << ",lower_margin" << joint
           << ",upper_margin" << joint << ",signed_margin" << joint;
  }
  output << ",minimum_margin_index,first_failed_index";
  for (int joint = 0; joint < 10; ++joint)
    output << ",raw_q" << joint << ",raw_dq" << joint;
  output << ",wheel_mesh_phase_left,wheel_mesh_phase_right";
  for (int index = 0; index < 3; ++index)
    output << ",base_p" << index;
  for (int index = 0; index < 4; ++index)
    output << ",base_q" << index;
  for (int index = 0; index < 3; ++index)
    output << ",base_rotvec" << index;
  for (int index = 0; index < 3; ++index)
    output << ",base_v" << index << ",base_omega" << index;
  for (int index = 0; index < 12; ++index)
    output << ",requested_wrench" << index << ",realized_wrench" << index
           << ",slack" << index << ",wrench_residual" << index;
  for (int joint = 0; joint < 6; ++joint)
    output << ",tau" << joint << ",tau_margin" << joint;
  output << ",maximum_normalized_slack\n";
}

void run(const std::string &model_path, const std::string &output_path,
         const std::string &case_id, const std::string &gain_id,
         double kp, double kd) {
  const bool known_case = case_id == "H0_minimal_hold" ||
      case_id == "H1_zero_ddxi_row" || case_id == "D_positive" ||
      case_id == "D_negative" || case_id.rfind("H2_", 0) == 0 ||
      case_id.rfind("tracking_step_", 0) == 0 ||
      case_id.rfind("tracking_ramp_", 0) == 0;
  if (!known_case) throw std::invalid_argument("unknown Phase35 case");
  std::ifstream probe(output_path);
  if (probe.good()) throw std::runtime_error("refusing to overwrite output");
  char error[1024]{};
  ModelPtr model(mj_loadXML(model_path.c_str(), nullptr, error, sizeof(error)));
  if (!model) throw std::runtime_error(error);
  if (model->nq != 17 || model->nv != 16 || model->nu != 6 ||
      std::abs(model->opt.timestep - 0.002) > 1e-12)
    throw std::runtime_error("full-3D invariant failed");
  DataPtr data(mj_makeData(model.get()));
  wheel_leg_mujoco::AdapterConfig adapter_config;
  adapter_config.floating_base = true;
  wheel_leg_mujoco::Adapter adapter(model.get(), adapter_config);
  adapter.reset(data.get());
  setInitialState(model.get(), data.get());
  const auto profile = case_id == "H0_minimal_hold"
      ? wheel_leg::WeightedWbcProfile::kPhase27Minimal
      : wheel_leg::WeightedWbcProfile::kPhase34XiTracking;
  wheel_leg::WeightedWbcController controller(profile);
  wheel_leg::NominalWbcModel wbc_model;
  wheel_leg::WheelPositionPlanner planner;
  auto state = adapter.extractState(data.get());
  const auto initial_model = wbc_model.evaluate(state);
  if (!initial_model.ok()) throw std::runtime_error("initial WBC model failed");
  const double common_initial = 0.5 *
      (initial_model.wheel_position_b_x_m[0] + initial_model.wheel_position_b_x_m[1]);
  const double differential_initial = 0.5 *
      (initial_model.wheel_position_b_x_m[1] - initial_model.wheel_position_b_x_m[0]);
  const auto reset_reference = planner.reset(common_initial, 0.5 *
      (initial_model.wheel_velocity_b_x_m_s[0] +
       initial_model.wheel_velocity_b_x_m_s[1]));
  if (std::abs(reset_reference.common_position_m - common_initial) > 1e-12)
    throw std::runtime_error("planner reset failed");

  std::array<int, 10> joint_ids{};
  for (int index = 0; index < 10; ++index)
    joint_ids[index] = requiredId(model.get(), mjOBJ_JOINT, kRawJointNames[index]);
#if defined(WHEEL_LEG_PHASE40_ANGLE_DOMAIN_DIAGNOSTIC) || \
    defined(WHEEL_LEG_PHASE41_PRODUCTION_REVALIDATION)
  const std::array<double, 2> initial_wheel_phase{
      data->qpos[model->jnt_qposadr[joint_ids[2]]],
      data->qpos[model->jnt_qposadr[joint_ids[5]]]};
  const auto initial_base_position = state.base_position_n_m;
  const auto initial_base_quaternion = state.q_n_from_b;
#endif
  const int floor_geom = requiredId(model.get(), mjOBJ_GEOM, "floor");
  std::array<int, 2> wheel_geom{};
  std::array<int, 2> wheel_body{};
  for (int side = 0; side < 2; ++side)
    wheel_geom[side] = requiredId(model.get(), mjOBJ_GEOM, kWheelGeomNames[side]);
  for (int side = 0; side < 2; ++side)
    wheel_body[side] = requiredId(model.get(), mjOBJ_BODY,
                                  side == 0 ? "left_wheel_body" : "right_wheel_body");
  const int base_site = requiredId(model.get(), mjOBJ_SITE, "base_control_frame");

  std::ofstream output(output_path);
  if (!output) throw std::runtime_error("cannot create output");
  output << std::setprecision(17);
  writeHeader(output);

  constexpr int kActivationTick = 50;
#if defined(WHEEL_LEG_PHASE40_ANGLE_DOMAIN_DIAGNOSTIC) || \
    defined(WHEEL_LEG_PHASE41_PRODUCTION_REVALIDATION)
  constexpr int kTotalTicks = 2000;
#else
  constexpr int kTotalTicks = 150;
#endif
  constexpr double kDt = 0.01;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  for (int tick = 0; tick < kTotalTicks; ++tick) {
    state = adapter.extractState(data.get());
    const auto workspace = wheel_leg::NominalWbcModel::inspectWorkspace(state);
    const auto measurement = wbc_model.evaluate(state);
    const double target_time = std::max(0.0, (tick - kActivationTick) * kDt);
    double offset = 0.0;
    if (case_id.rfind("tracking_step_", 0) == 0 && tick >= kActivationTick)
      offset = 0.005;
    if (case_id.rfind("tracking_ramp_", 0) == 0)
      offset = 0.02 * std::min(target_time, 0.25);
    const auto planned = planner.step(common_initial + offset, kDt);

    double common = nan, common_velocity = nan, differential = nan;
    double differential_velocity = nan;
    if (measurement.ok()) {
      common = 0.5 * (measurement.wheel_position_b_x_m[0] +
                      measurement.wheel_position_b_x_m[1]);
      common_velocity = 0.5 * (measurement.wheel_velocity_b_x_m_s[0] +
                               measurement.wheel_velocity_b_x_m_s[1]);
      differential = 0.5 * (measurement.wheel_position_b_x_m[1] -
                            measurement.wheel_position_b_x_m[0]);
      differential_velocity = 0.5 *
          (measurement.wheel_velocity_b_x_m_s[1] -
           measurement.wheel_velocity_b_x_m_s[0]);
    }
    double common_acceleration = 0.0;
    double differential_acceleration = 0.0;
    if (case_id == "D_positive" || case_id == "D_negative") {
      const double sign = case_id == "D_positive" ? 1.0 : -1.0;
      if (tick >= 50 && tick < 60) common_acceleration = sign * 0.25;
      if (tick >= 60 && tick < 70) common_acceleration = -sign * 0.25;
    } else if (case_id.rfind("H2_", 0) == 0) {
      common_acceleration = planned.common_acceleration_m_s2 +
          kp * (common_initial - common) - kd * common_velocity;
      differential_acceleration =
          kp * (differential_initial - differential) - kd * differential_velocity;
    } else if (case_id.rfind("tracking_", 0) == 0) {
      common_acceleration = planned.common_acceleration_m_s2 +
          kp * (planned.common_position_m - common) +
          kd * (planned.common_velocity_m_s - common_velocity);
      differential_acceleration =
          kp * (differential_initial - differential) - kd * differential_velocity;
    }
    auto reference = equilibriumReference();
    reference.wheel_longitudinal_acceleration_m_s2 <<
        common_acceleration - differential_acceleration,
        common_acceleration + differential_acceleration;

    wheel_leg::WeightedWbcController::Result result;
    double wbc_time = 0.0;
    if (measurement.ok()) {
      const auto start = std::chrono::steady_clock::now();
      result = controller.step(state, reference);
      wbc_time = std::chrono::duration<double>(
          std::chrono::steady_clock::now() - start).count();
    } else {
      result.model_status = measurement.status;
      result.status = wheel_leg::WeightedWbcController::Status::kModelRejected;
    }

    std::array<double, 2> normal_load{};
    for (int contact = 0; contact < data->ncon; ++contact) {
      double force[6]{};
      mj_contactForce(model.get(), data.get(), contact, force);
      for (int side = 0; side < 2; ++side)
        if (pairMatches(data->contact[contact].geom1, data->contact[contact].geom2,
                        wheel_geom[side], floor_geom))
          normal_load[side] += std::abs(force[0]);
    }
    Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> base_rotation(
        data->site_xmat + 9 * base_site);
    const Eigen::Vector3d base_position(data->site_xpos + 3 * base_site);
    const Eigen::Vector3d base_velocity(
        state.base_linear_velocity_n_m_s.data());
    const Eigen::Vector3d base_omega(
        state.base_angular_velocity_n_rad_s.data());
    std::array<Eigen::Vector3d, 2> raw_wheel_position{};
    std::array<Eigen::Vector3d, 2> raw_wheel_velocity{};
    for (int side = 0; side < 2; ++side) {
      std::array<double, 48> jacobian_position{};
      std::array<double, 48> jacobian_rotation{};
      mj_jacBody(model.get(), data.get(), jacobian_position.data(),
                 jacobian_rotation.data(), wheel_body[side]);
      Eigen::Vector3d wheel_velocity_world;
      mju_mulMatVec(wheel_velocity_world.data(), jacobian_position.data(),
                    data->qvel, 3, model->nv);
      const Eigen::Vector3d relative_world(
          data->xpos[3 * wheel_body[side]] - base_position.x(),
          data->xpos[3 * wheel_body[side] + 1] - base_position.y(),
          data->xpos[3 * wheel_body[side] + 2] - base_position.z());
      raw_wheel_position[side] = base_rotation.transpose() * relative_world;
      raw_wheel_velocity[side] = base_rotation.transpose() *
          (wheel_velocity_world - base_velocity - base_omega.cross(relative_world));
    }

    output << tick << ',' << data->time << ',' << case_id << ',' << gain_id << ','
           << (tick >= kActivationTick) << ',' << static_cast<int>(measurement.status)
           << ',' << static_cast<int>(result.status) << ','
           << static_cast<int>(result.solver_status) << ',' << result.iterations << ','
           << wbc_time << ',' << result.hard_violation << ',' << result.primal_residual
           << ',' << result.dual_residual << ',' << result.stationarity_residual << ','
           << (state.contact_state[0] == wheel_leg::ContactState::kContact) << ','
           << (state.contact_state[1] == wheel_leg::ContactState::kContact) << ','
           << normal_load[0] << ',' << normal_load[1] << ','
           << planned.common_position_m << ',' << planned.common_velocity_m_s << ','
           << planned.common_acceleration_m_s2 << ','
           << reference.wheel_longitudinal_acceleration_m_s2[0] << ','
           << reference.wheel_longitudinal_acceleration_m_s2[1] << ','
           << (measurement.ok() ? measurement.wheel_position_b_x_m[0] : nan) << ','
           << (measurement.ok() ? measurement.wheel_position_b_x_m[1] : nan) << ','
           << (measurement.ok() ? measurement.wheel_velocity_b_x_m_s[0] : nan) << ','
           << (measurement.ok() ? measurement.wheel_velocity_b_x_m_s[1] : nan) << ','
           << common << ',' << differential << ',' << common_velocity << ','
           << differential_velocity << ','
           << (measurement.ok() ? measurement.wheel_position_b_z_m[0] : nan) << ','
           << (measurement.ok() ? measurement.wheel_position_b_z_m[1] : nan) << ','
           << (measurement.ok() ? measurement.wheel_velocity_b_z_m_s[0] : nan) << ','
           << (measurement.ok() ? measurement.wheel_velocity_b_z_m_s[1] : nan);
    for (int side = 0; side < 2; ++side)
      for (int axis = 0; axis < 3; ++axis)
        output << ',' << raw_wheel_position[side][axis] << ','
               << raw_wheel_velocity[side][axis];
    output << ','
           << (result.ok() ? result.wheel_longitudinal_acceleration_m_s2[0] : nan)
           << ','
           << (result.ok() ? result.wheel_longitudinal_acceleration_m_s2[1] : nan);
    for (int joint = 0; joint < 6; ++joint) {
      const auto &entry = workspace.joint[joint];
      output << ',' << state.joint_position_rad[joint] << ','
             << state.joint_velocity_rad_s[joint] << ',' << entry.equilibrium_rad
             << ',' << entry.delta_rad << ',' << entry.lower_margin_rad << ','
             << entry.upper_margin_rad << ',' << entry.signed_margin_rad;
    }
    output << ',' << workspace.minimum_margin_index << ','
           << workspace.first_failed_index;
    for (int joint = 0; joint < 10; ++joint) {
      const int id = joint_ids[joint];
      output << ',' << data->qpos[model->jnt_qposadr[id]] << ','
             << data->qvel[model->jnt_dofadr[id]];
    }
    output << ',' << data->qpos[model->jnt_qposadr[joint_ids[2]]] << ','
           << data->qpos[model->jnt_qposadr[joint_ids[5]]];
    for (double value : state.base_position_n_m) output << ',' << value;
    for (double value : state.q_n_from_b) output << ',' << value;
    const Eigen::Quaterniond quaternion(
        state.q_n_from_b[0], state.q_n_from_b[1], state.q_n_from_b[2],
        state.q_n_from_b[3]);
    const Eigen::AngleAxisd angle_axis(quaternion);
    const Eigen::Vector3d rotation_vector = angle_axis.angle() * angle_axis.axis();
    for (double value : rotation_vector) output << ',' << value;
    for (int index = 0; index < 3; ++index)
      output << ',' << state.base_linear_velocity_n_m_s[index] << ','
             << state.base_angular_velocity_n_rad_s[index];
    for (int index = 0; index < 12; ++index)
      output << ',' << reference.interaction_wrench_flu[index] << ','
             << (result.ok() ? result.realized_interaction_wrench_flu[index] : nan)
             << ',' << (result.ok() ? result.signed_interaction_slack_flu[index] : nan)
             << ',' << (result.ok() ? result.interaction_wrench_residual_flu[index] : nan);
    for (int joint = 0; joint < 6; ++joint) {
      const double torque = result.ok() ? result.torque_nm[joint] : nan;
      output << ',' << torque << ','
             << (result.ok() ? kTorqueLimit[joint] - std::abs(torque) : nan);
    }
    output << ',' << (result.ok() ? result.maximum_normalized_slack : nan) << '\n';

    if (!measurement.ok()) break;
#if defined(WHEEL_LEG_PHASE40_ANGLE_DOMAIN_DIAGNOSTIC) || \
    defined(WHEEL_LEG_PHASE41_PRODUCTION_REVALIDATION)
    if (!result.ok()) break;
    const double base_position_change = std::sqrt(
        std::pow(state.base_position_n_m[0] - initial_base_position[0], 2) +
        std::pow(state.base_position_n_m[1] - initial_base_position[1], 2) +
        std::pow(state.base_position_n_m[2] - initial_base_position[2], 2));
    double quaternion_dot = 0.0;
    for (int index = 0; index < 4; ++index)
      quaternion_dot += state.q_n_from_b[index] * initial_base_quaternion[index];
    const double base_rotation_change =
        2.0 * std::acos(std::min(1.0, std::abs(quaternion_dot)));
    const double base_linear_speed = std::sqrt(
        std::pow(state.base_linear_velocity_n_m_s[0], 2) +
        std::pow(state.base_linear_velocity_n_m_s[1], 2) +
        std::pow(state.base_linear_velocity_n_m_s[2], 2));
    const double base_angular_speed = std::sqrt(
        std::pow(state.base_angular_velocity_n_rad_s[0], 2) +
        std::pow(state.base_angular_velocity_n_rad_s[1], 2) +
        std::pow(state.base_angular_velocity_n_rad_s[2], 2));
    double minimum_torque_margin = std::numeric_limits<double>::infinity();
    for (int joint = 0; joint < 6; ++joint)
      minimum_torque_margin = std::min(
          minimum_torque_margin,
          kTorqueLimit[joint] - std::abs(result.torque_nm[joint]));
    const bool independent_failure =
        state.contact_state[0] != wheel_leg::ContactState::kContact ||
        state.contact_state[1] != wheel_leg::ContactState::kContact ||
        result.hard_violation > 1.0e-7 ||
        result.maximum_normalized_slack > 0.05 ||
        minimum_torque_margin < -1.0e-10 || base_position_change > 0.1 ||
        base_rotation_change > 0.35 || base_linear_speed > 2.0 ||
        base_angular_speed > 5.0;
    if (independent_failure) break;
    double maximum_wheel_rotation = 0.0;
    for (int side = 0; side < 2; ++side) {
      maximum_wheel_rotation = std::max(
          maximum_wheel_rotation,
          std::abs(data->qpos[model->jnt_qposadr[joint_ids[side * 3 + 2]]] -
                   initial_wheel_phase[side]));
    }
    if (maximum_wheel_rotation >= 6.0 * std::acos(-1.0)) break;
#else
    if (!result.ok()) throw std::runtime_error("WBC solve failed before workspace gate");
#endif
    for (int joint = 0; joint < 6; ++joint)
      data->ctrl[joint] = -result.torque_nm[joint];
    for (int substep = 0; substep < 5; ++substep) mj_step(model.get(), data.get());
  }
}

}  // namespace

int main(int argc, char **argv) {
  if (argc != 7) {
#ifdef WHEEL_LEG_PHASE41_PRODUCTION_REVALIDATION
    std::cerr << "usage: phase41_workspace_contract_loop MODEL OUTPUT CASE GAIN KP KD\n";
#elif defined(WHEEL_LEG_PHASE40_ANGLE_DOMAIN_DIAGNOSTIC)
    std::cerr << "usage: phase40_angle_domain_loop MODEL OUTPUT CASE GAIN KP KD\n";
#else
    std::cerr << "usage: phase35_workspace_attribution_loop MODEL OUTPUT CASE GAIN KP KD\n";
#endif
    return 1;
  }
  try {
    run(argv[1], argv[2], argv[3], argv[4], std::stod(argv[5]), std::stod(argv[6]));
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 2;
  }
  return 0;
}
