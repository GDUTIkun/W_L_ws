#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "mujoco/mujoco.h"
#include "wheel_leg_core/weighted_wbc_controller.hpp"
#include "wheel_leg_core/wheel_position_planner.hpp"
#include "wheel_leg_mujoco/adapter.hpp"

namespace {

struct ModelDeleter { void operator()(mjModel *value) const { mj_deleteModel(value); } };
struct DataDeleter { void operator()(mjData *value) const { mj_deleteData(value); } };
using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;

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

void run(const std::string &model_path, const std::string &output_path,
         const std::string &profile, double kp, double kd) {
  if (profile != "step" && profile != "ramp")
    throw std::invalid_argument("profile must be step or ramp");
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
  wheel_leg::WeightedWbcController controller(
      wheel_leg::WeightedWbcProfile::kPhase34XiTracking);
  wheel_leg::NominalWbcModel wbc_model;
  wheel_leg::WheelPositionPlanner planner;
  auto reference = equilibriumReference();
  auto state = adapter.extractState(data.get());
  const auto initial_model = wbc_model.evaluate(state);
  if (!initial_model.ok()) throw std::runtime_error("initial WBC model failed");
  const double common_initial = 0.5 * (
      initial_model.wheel_position_b_x_m[0] +
      initial_model.wheel_position_b_x_m[1]);
  const double differential_reference =
      0.5 * (initial_model.wheel_position_b_x_m[1] -
             initial_model.wheel_position_b_x_m[0]);
  const auto reset_reference = planner.reset(common_initial, 0.5 * (
      initial_model.wheel_velocity_b_x_m_s[0] +
      initial_model.wheel_velocity_b_x_m_s[1]));
  if (!planner.initialized() ||
      std::abs(reset_reference.common_position_m - common_initial) > 1e-12)
    throw std::runtime_error("planner reset failed");

  std::ofstream output(output_path);
  if (!output) throw std::runtime_error("cannot create output");
  output << std::setprecision(17)
         << "tick,time_s,profile,status,solver_status,hard,wbc_time_s,contact_left,contact_right"
         << ",planner_xi,planner_dxi,planner_ddxi,xi_common,dxi_common,xi_differential"
         << ",dxi_differential,common_error,differential_error,max_normalized_slack"
         << ",max_wrench_residual,max_torque";
  for (int index = 0; index < 6; ++index) output << ",tau" << index;
  output << '\n';

  constexpr int kPreTicks = 50;
  constexpr int kTotalTicks = 150;
  constexpr double kControlPeriod = 0.01;
  constexpr double kStep = 0.005;
  constexpr double kRampRate = 0.02;
  constexpr double kRampDuration = 0.25;
  for (int tick = 0; tick < kTotalTicks; ++tick) {
    state = adapter.extractState(data.get());
    const double target_time = std::max(0.0, (tick - kPreTicks) * kControlPeriod);
    const double offset = profile == "step"
        ? (tick >= kPreTicks ? kStep : 0.0)
        : kRampRate * std::min(target_time, kRampDuration);
    const auto planned = planner.step(common_initial + offset, kControlPeriod);

    const auto measurement = wbc_model.evaluate(state);
    if (!measurement.ok())
      throw std::runtime_error(
          "measurement WBC model failed: status=" +
          std::to_string(static_cast<int>(measurement.status)));
    const double common = 0.5 * (
        measurement.wheel_position_b_x_m[0] +
        measurement.wheel_position_b_x_m[1]);
    const double common_velocity = 0.5 * (
        measurement.wheel_velocity_b_x_m_s[0] +
        measurement.wheel_velocity_b_x_m_s[1]);
    const double differential =
        0.5 * (measurement.wheel_position_b_x_m[1] -
               measurement.wheel_position_b_x_m[0]);
    const double differential_velocity =
        0.5 * (measurement.wheel_velocity_b_x_m_s[1] -
               measurement.wheel_velocity_b_x_m_s[0]);
    const double common_acceleration = planned.common_acceleration_m_s2 +
        kp * (planned.common_position_m - common) +
        kd * (planned.common_velocity_m_s - common_velocity);
    const double differential_acceleration =
        kp * (differential_reference - differential) - kd * differential_velocity;
    reference = equilibriumReference();
    reference.wheel_longitudinal_acceleration_m_s2 <<
        common_acceleration - differential_acceleration,
        common_acceleration + differential_acceleration;
    const auto start = std::chrono::steady_clock::now();
    const auto result = controller.step(state, reference);
    const double wbc_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start).count();
    if (!result.ok()) throw std::runtime_error("tracking WBC solve failed");
    for (int joint = 0; joint < 6; ++joint) data->ctrl[joint] = -result.torque_nm[joint];
    for (int substep = 0; substep < 5; ++substep) mj_step(model.get(), data.get());
    const double max_torque = *std::max_element(
        result.torque_nm.begin(), result.torque_nm.end(),
        [](double first, double second) { return std::abs(first) < std::abs(second); });
    output << tick << ',' << data->time << ',' << profile << ','
           << static_cast<int>(result.status) << ','
           << static_cast<int>(result.solver_status) << ',' << result.hard_violation
           << ',' << wbc_time << ','
           << (state.contact_state[0] == wheel_leg::ContactState::kContact) << ','
           << (state.contact_state[1] == wheel_leg::ContactState::kContact) << ','
           << planned.common_position_m << ',' << planned.common_velocity_m_s << ','
           << planned.common_acceleration_m_s2 << ',' << common << ','
           << common_velocity << ',' << differential << ',' << differential_velocity
           << ',' << planned.common_position_m - common << ','
           << differential_reference - differential << ','
           << result.maximum_normalized_slack << ','
           << result.interaction_wrench_residual_flu.cwiseAbs().maxCoeff() << ','
           << std::abs(max_torque);
    for (double torque : result.torque_nm) output << ',' << torque;
    output << '\n';
  }
}

}  // namespace

int main(int argc, char **argv) {
  if (argc != 6) {
    std::cerr << "usage: phase34_xi_tracking_loop MODEL OUTPUT step|ramp KP KD\n";
    return 1;
  }
  try {
    run(argv[1], argv[2], argv[3], std::stod(argv[4]), std::stod(argv[5]));
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 2;
  }
  return 0;
}
