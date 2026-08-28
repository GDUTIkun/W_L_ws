#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "mujoco/mujoco.h"
#include "wheel_leg_core/controller_core.hpp"
#include "wheel_leg_mujoco/adapter.hpp"

namespace {

struct ModelDeleter {
  void operator()(mjModel *value) const { mj_deleteModel(value); }
};
struct DataDeleter {
  void operator()(mjData *value) const { mj_deleteData(value); }
};
using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;

struct Options {
  std::string model_path;
  std::string control_path;
  std::string plant_path;
  std::string scenario{"hold"};
  int episodes{1};
  int ticks{100};
  int fault_tick{100};
  int disturbance_start_tick{-1};
  int disturbance_ticks{0};
  std::array<double, 3> force{};
  std::array<double, 3> moment{};
  std::array<double, 8> initial_state{};
  std::array<double, 4> leg_perturbation{};
  std::array<double, 9> equilibrium{
      -0.34332947374181766, 0.5693992271789607,
      -0.35472149355205396, 0.5694045089964002,
      0.34771766403249466, -0.572545089643551,
      -0.5729875480645877, 0.5725979309569537,
      -0.5730345859812999};
  wheel_leg::JointVector torque_limit{10, 10, 2, 10, 10, 2};
};

int requiredId(const mjModel *model, mjtObj type, const char *name) {
  const int id = mj_name2id(model, type, name);
  if (id < 0)
    throw std::runtime_error(std::string("Missing MuJoCo object: ") + name);
  return id;
}

template <std::size_t Size>
std::array<double, Size> parseVector(const std::string &text) {
  std::array<double, Size> result{};
  std::istringstream input(text);
  std::string item;
  std::size_t index = 0;
  while (std::getline(input, item, ',')) {
    if (index >= Size) throw std::invalid_argument("too many vector elements");
    result[index++] = std::stod(item);
  }
  if (index != Size || !std::all_of(result.begin(), result.end(),
      [](double value) { return std::isfinite(value); })) {
    throw std::invalid_argument("invalid vector");
  }
  return result;
}

Options parseOptions(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; index += 2) {
    if (index + 1 >= argc) throw std::invalid_argument("missing option value");
    const std::string argument = argv[index];
    const std::string value = argv[index + 1];
    if (argument == "--model") options.model_path = value;
    else if (argument == "--control-output") options.control_path = value;
    else if (argument == "--plant-output") options.plant_path = value;
    else if (argument == "--scenario") options.scenario = value;
    else if (argument == "--episodes") options.episodes = std::stoi(value);
    else if (argument == "--ticks") options.ticks = std::stoi(value);
    else if (argument == "--fault-tick") options.fault_tick = std::stoi(value);
    else if (argument == "--disturbance-start-tick") options.disturbance_start_tick = std::stoi(value);
    else if (argument == "--disturbance-ticks") options.disturbance_ticks = std::stoi(value);
    else if (argument == "--force") options.force = parseVector<3>(value);
    else if (argument == "--moment") options.moment = parseVector<3>(value);
    else if (argument == "--initial-state") options.initial_state = parseVector<8>(value);
    else if (argument == "--leg-perturbation") options.leg_perturbation = parseVector<4>(value);
    else if (argument == "--equilibrium") options.equilibrium = parseVector<9>(value);
    else if (argument == "--torque-limit") options.torque_limit = parseVector<6>(value);
    else throw std::invalid_argument("unknown option: " + argument);
  }
  const std::array<std::string, 6> scenarios{
      "hold", "contact_loss_left", "contact_loss_right", "invalid", "nonmonotonic", "timing"};
  if (options.model_path.empty() || options.control_path.empty() || options.plant_path.empty())
    throw std::invalid_argument("--model --control-output --plant-output required");
  if (options.episodes <= 0 || options.ticks <= 0 ||
      std::find(scenarios.begin(), scenarios.end(), options.scenario) == scenarios.end())
    throw std::invalid_argument("invalid run options");
  return options;
}

template <typename Range>
void writeValues(std::ostream &output, const Range &values) {
  for (const auto &value : values) output << ',' << value;
}

struct PlantMetrics {
  std::array<double, 2> normal_load_n{};
  double maximum_penetration_m{0.0};
  double maximum_abs_rolling_slip_m_s{0.0};
  double maximum_abs_lateral_slip_m_s{0.0};
  double closure_residual_m{0.0};
};

PlantMetrics plantMetrics(const mjModel *model, const mjData *data) {
  PlantMetrics result;
  const int floor = requiredId(model, mjOBJ_GEOM, "floor");
  const std::array<int, 2> wheel_geom{
      requiredId(model, mjOBJ_GEOM, "left_wheel_collision"),
      requiredId(model, mjOBJ_GEOM, "right_wheel_collision")};
  const std::array<int, 2> wheel_body{
      requiredId(model, mjOBJ_BODY, "left_wheel_body"),
      requiredId(model, mjOBJ_BODY, "right_wheel_body")};
  const std::array<int, 2> wheel_dof{
      model->jnt_dofadr[requiredId(model, mjOBJ_JOINT, "left_wheel_joint")],
      model->jnt_dofadr[requiredId(model, mjOBJ_JOINT, "right_wheel_joint")]};
  for (int contact_index = 0; contact_index < data->ncon; ++contact_index) {
    const auto &contact = data->contact[contact_index];
    for (std::size_t side = 0; side < wheel_geom.size(); ++side) {
      const bool floor_contact =
          (contact.geom1 == floor && contact.geom2 == wheel_geom[side]) ||
          (contact.geom2 == floor && contact.geom1 == wheel_geom[side]);
      if (!floor_contact) continue;
      std::array<mjtNum, 6> local_force{};
      std::array<mjtNum, 3> world_force{};
      mj_contactForce(model, data, contact_index, local_force.data());
      mju_mulMatTVec(world_force.data(), contact.frame, local_force.data(), 3, 3);
      if (contact.geom2 != wheel_geom[side])
        for (double &value : world_force) value = -value;
      result.normal_load_n[side] += world_force[2];
      result.maximum_penetration_m = std::max(
          result.maximum_penetration_m, std::max(0.0, -contact.dist));
    }
  }
  for (std::size_t side = 0; side < wheel_body.size(); ++side) {
    std::array<mjtNum, 6> velocity{};
    mj_objectVelocity(model, data, mjOBJ_BODY, wheel_body[side], velocity.data(), 0);
    result.maximum_abs_rolling_slip_m_s = std::max(
        result.maximum_abs_rolling_slip_m_s,
        std::abs(velocity[3] - 0.05 * data->qvel[wheel_dof[side]]));
    result.maximum_abs_lateral_slip_m_s = std::max(
        result.maximum_abs_lateral_slip_m_s, std::abs(velocity[4]));
  }
  for (const char *side : {"left", "right"}) {
    const int first = requiredId(
        model, mjOBJ_SITE, (std::string(side) + "_connect2_site").c_str());
    const int second = requiredId(
        model, mjOBJ_SITE, (std::string(side) + "_calf_site").c_str());
    double squared_distance = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
      const double difference =
          data->site_xpos[3 * first + axis] - data->site_xpos[3 * second + axis];
      squared_distance += difference * difference;
    }
    result.closure_residual_m = std::max(
        result.closure_residual_m, std::sqrt(squared_distance));
  }
  return result;
}

bool bilateralContact(const mjModel *model, const mjData *data) {
  const int floor = requiredId(model, mjOBJ_GEOM, "floor");
  const std::array<int, 2> wheel{
      requiredId(model, mjOBJ_GEOM, "left_wheel_collision"),
      requiredId(model, mjOBJ_GEOM, "right_wheel_collision")};
  std::array<bool, 2> found{};
  for (int contact_index = 0; contact_index < data->ncon; ++contact_index) {
    const auto &contact = data->contact[contact_index];
    for (std::size_t side = 0; side < wheel.size(); ++side) {
      found[side] = found[side] ||
          ((contact.geom1 == floor && contact.geom2 == wheel[side]) ||
           (contact.geom2 == floor && contact.geom1 == wheel[side]));
    }
  }
  return found[0] && found[1];
}

void setInitialState(const mjModel *model, mjData *data, const Options &options) {
  // Adapter::reset already ran mj_resetData and disabled the scene base_weld
  // equality for floating-base runs; resetting here would re-enable the weld
  // and anchor the base at the XML qpos0 height.
  data->qpos[2] = options.equilibrium[4];
  data->qpos[3] = 1.0;
  const std::array<const char *, 4> active{
      "right_hip_joint", "right_knee_joint", "left_hip_joint", "left_knee_joint"};
  const std::array<const char *, 4> passive{
      "right_connect1_joint", "right_connect2_joint",
      "left_connect1_joint", "left_connect2_joint"};
  for (std::size_t index = 0; index < active.size(); ++index) {
    data->qpos[model->jnt_qposadr[requiredId(model, mjOBJ_JOINT, active[index])]] =
        options.equilibrium[index];
    data->qpos[model->jnt_qposadr[requiredId(model, mjOBJ_JOINT, passive[index])]] =
        options.equilibrium[index + 5];
  }
  mj_forward(model, data);
  const int site = requiredId(model, mjOBJ_SITE, "base_control_frame");
  // This is intentionally captured before the initial rotation changes the site pose.
  const double equilibrium_site_x = data->site_xpos[3 * site];
  const std::array<double, 3> rotation{
      options.initial_state[4], options.initial_state[2], options.initial_state[6]};
  const double angle = std::sqrt(
      rotation[0] * rotation[0] + rotation[1] * rotation[1] + rotation[2] * rotation[2]);
  if (angle > 0.0) {
    data->qpos[3] = std::cos(angle / 2.0);
    for (int axis = 0; axis < 3; ++axis)
      data->qpos[4 + axis] = std::sin(angle / 2.0) * rotation[axis] / angle;
  }
  mj_forward(model, data);
  data->qpos[0] += equilibrium_site_x + options.initial_state[0] - data->site_xpos[3 * site];
  data->qvel[3] = options.initial_state[5];
  data->qvel[4] = options.initial_state[3];
  data->qvel[5] = options.initial_state[7];
  mj_forward(model, data);
  std::vector<mjtNum> jacobian_position(3 * model->nv);
  mj_jacSite(model, data, jacobian_position.data(), nullptr, site);
  double site_vx = 0.0;
  for (int dof = 0; dof < model->nv; ++dof)
    site_vx += jacobian_position[dof] * data->qvel[dof];
  data->qvel[0] += options.initial_state[1] - site_vx;
  const std::array<const char *, 4> leg{
      "left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"};
  for (std::size_t index = 0; index < leg.size(); ++index) {
    data->qpos[model->jnt_qposadr[requiredId(model, mjOBJ_JOINT, leg[index])]] -=
        options.leg_perturbation[index];
  }
  mj_forward(model, data);
  if (!bilateralContact(model, data))
    throw std::runtime_error("Initial state outside bilateral contact");
}

void writeHeaders(std::ofstream &control, std::ofstream &plant) {
  control << "scenario,episode,tick,pre_step_plant_time_s,source_ns,command_source_ns,receipt_ns,dt_s,status,latch,accepted,contact_left,contact_right,weighted_status,model_status,solver_status,iterations,primal,dual,stationarity,hard,reconstruction_iterations,closure_residual,core_step_ns,zoh_diff";
  for (int index = 0; index < 3; ++index) control << ",base_p" << index;
  for (int index = 0; index < 4; ++index) control << ",quat" << index;
  for (int index = 0; index < 3; ++index) control << ",base_v" << index;
  for (int index = 0; index < 3; ++index) control << ",base_w" << index;
  for (int index = 0; index < 6; ++index) control << ",q" << index;
  for (int index = 0; index < 6; ++index) control << ",dq" << index;
  for (int index = 0; index < 21; ++index) control << ",reference" << index;
  for (int index = 0; index < 42; ++index) control << ",z" << index;
  for (int index = 0; index < 7; ++index) control << ",task_residual" << index;
  for (int index = 0; index < 7; ++index) control << ",task_cost" << index;
  control << ",max_normalized_slack";
  for (int index = 0; index < 6; ++index) control << ",raw_tau" << index;
  for (int index = 0; index < 6; ++index) control << ",command_tau" << index;
  for (int index = 0; index < 6; ++index) control << ",held_ctrl" << index;
  control << '\n';
  plant << "scenario,episode,control_tick,physics_substep,time_s,disturbance,force_x,force_y,force_z,moment_x,moment_y,moment_z,left_normal_load_n,right_normal_load_n,penetration_m,rolling_slip_m_s,lateral_slip_m_s,closure_residual_m";
  for (int index = 0; index < 17; ++index) plant << ",qpos" << index;
  for (int index = 0; index < 16; ++index) plant << ",qvel" << index;
  for (int index = 0; index < 6; ++index) plant << ",ctrl" << index;
  plant << '\n';
}

void writePlantRow(std::ofstream &plant, const Options &options, int episode, int tick,
                   int substep, const mjData *data, bool disturbed,
                   const PlantMetrics &metrics) {
  plant << options.scenario << ',' << episode << ',' << tick << ',' << substep
        << ',' << data->time << ',' << disturbed;
  if (disturbed) {
    writeValues(plant, options.force);
    writeValues(plant, options.moment);
  } else {
    writeValues(plant, std::array<double, 3>{});
    writeValues(plant, std::array<double, 3>{});
  }
  writeValues(plant, metrics.normal_load_n);
  plant << ',' << metrics.maximum_penetration_m
        << ',' << metrics.maximum_abs_rolling_slip_m_s
        << ',' << metrics.maximum_abs_lateral_slip_m_s
        << ',' << metrics.closure_residual_m;
  for (int index = 0; index < 17; ++index) plant << ',' << data->qpos[index];
  for (int index = 0; index < 16; ++index) plant << ',' << data->qvel[index];
  for (int actuator = 0; actuator < 6; ++actuator) plant << ',' << data->ctrl[actuator];
  plant << '\n';
}

void writeControlRow(std::ofstream &control, const Options &options, int episode, int tick,
                     double pre_step_plant_time_s, const wheel_leg::RobotState &state,
                     const wheel_leg::StepResult &result, std::uint64_t receipt_time_ns,
                     std::int64_t core_step_ns, double zoh_difference,
                     const std::array<double, 6> &held_control, bool accepted) {
  control << options.scenario << ',' << episode << ',' << tick << ','
          << pre_step_plant_time_s << ',' << state.sample_time_ns << ','
          << result.command.source_sample_time_ns << ',' << receipt_time_ns << ','
          << result.dt_s << ',' << static_cast<int>(result.status) << ','
          << result.safety_latched << ',' << accepted << ','
          << static_cast<int>(state.contact_state[0]) << ','
          << static_cast<int>(state.contact_state[1]) << ','
          << static_cast<int>(result.weighted_wbc_status) << ','
          << static_cast<int>(result.weighted_wbc_model_status) << ','
          << static_cast<int>(result.weighted_wbc_solver_status) << ','
          << result.weighted_wbc_iterations << ',' << result.weighted_wbc_primal_residual
          << ',' << result.weighted_wbc_dual_residual << ','
          << result.weighted_wbc_stationarity_residual << ','
          << result.weighted_wbc_hard_violation << ','
          << result.weighted_wbc_model_diagnostics.reconstruction_iterations << ','
          << result.weighted_wbc_model_diagnostics.closure_residual_m << ','
          << core_step_ns << ',' << zoh_difference;
  writeValues(control, state.base_position_n_m);
  writeValues(control, state.q_n_from_b);
  writeValues(control, state.base_linear_velocity_n_m_s);
  writeValues(control, state.base_angular_velocity_n_rad_s);
  writeValues(control, state.joint_position_rad);
  writeValues(control, state.joint_velocity_rad_s);
  control << ',' << result.weighted_wbc_reference.base_x_acceleration_m_s2
          << ',' << result.weighted_wbc_reference.base_height_acceleration_m_s2;
  writeValues(control, result.weighted_wbc_reference.orientation_acceleration_rad_s2);
  writeValues(control, result.weighted_wbc_reference.leg_acceleration_rad_s2);
  writeValues(control, result.weighted_wbc_reference.interaction_wrench_flu);
  for (int index = 0; index < 42; ++index)
    control << ',' << result.weighted_wbc_physical_solution[index];
  writeValues(control, result.weighted_wbc_task_max_abs_normalized_residual);
  writeValues(control, result.weighted_wbc_task_normalized_squared_cost);
  control << ',' << result.weighted_wbc_maximum_normalized_slack;
  writeValues(control, result.tau_raw_nm);
  writeValues(control, result.command.joint_torque_nm);
  writeValues(control, held_control);
  control << '\n';
}

void run(const Options &options) {
  if (std::filesystem::exists(options.control_path) || std::filesystem::exists(options.plant_path))
    throw std::runtime_error("Refusing to overwrite output");
  char error[1024]{};
  ModelPtr model(mj_loadXML(options.model_path.c_str(), nullptr, error, sizeof(error)));
  if (!model) throw std::runtime_error(error);
  if (model->nq != 17 || model->nv != 16 || model->nu != 6 ||
      std::abs(model->opt.timestep - 0.002) > 1e-12)
    throw std::runtime_error("Full-3D invariant failed");
  DataPtr data(mj_makeData(model.get()));
  wheel_leg_mujoco::AdapterConfig adapter_config;
  adapter_config.command_enabled = true;
  adapter_config.floating_base = true;
  wheel_leg_mujoco::Adapter adapter(model.get(), adapter_config);
  wheel_leg::ControllerConfig config;
  config.mode = wheel_leg::ControllerMode::kWeightedWbc;
  config.torque_limit_nm = options.torque_limit;
  config.weighted_wbc = wheel_leg::currentNominalWeightedWbcConfig();
  wheel_leg::ControllerCore controller;
  if (!controller.configure(config)) throw std::runtime_error("WBC configuration failed");
  std::ofstream control(options.control_path);
  std::ofstream plant(options.plant_path);
  if (!control || !plant) throw std::runtime_error("output open failed");
  control << std::setprecision(17);
  plant << std::setprecision(17);
  writeHeaders(control, plant);
  const int base_body = requiredId(model.get(), mjOBJ_BODY, "base_body");
  constexpr std::uint64_t kControlPeriodNs = 10'000'000U;
  constexpr int kPhysicsSubstepsPerControl = 5;
  for (int episode = 0; episode < options.episodes; ++episode) {
    adapter.reset(data.get());
    controller.reset();
    setInitialState(model.get(), data.get(), options);
    std::uint64_t previous_source_time_ns = 0;
    for (int tick = 0; tick < options.ticks; ++tick) {
      const double pre_step_plant_time_s = data->time;
      auto state = adapter.extractState(data.get());
      if (tick == options.fault_tick) {
        if (options.scenario == "contact_loss_left")
          state.contact_state[0] = wheel_leg::ContactState::kNoContact;
        else if (options.scenario == "contact_loss_right")
          state.contact_state[1] = wheel_leg::ContactState::kNoContact;
        else if (options.scenario == "invalid")
          state.q_n_from_b[0] = std::numeric_limits<double>::quiet_NaN();
        else if (options.scenario == "nonmonotonic")
          state.sample_time_ns = previous_source_time_ns;
        else if (options.scenario == "timing")
          state.sample_time_ns += 1'000'000U;
      }
      const auto start = std::chrono::steady_clock::now();
      const auto result = controller.step(state);
      const auto core_step_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - start).count();
      const std::uint64_t receipt_time_ns = static_cast<std::uint64_t>(tick) * kControlPeriodNs;
      const bool accepted = adapter.acceptCommand(
          result.command, receipt_time_ns,
          wheel_leg_mujoco::Adapter::simulationTimeNs(data->time));
      adapter.writeControls(data.get(), receipt_time_ns);
      std::array<double, 6> held_control{};
      std::copy_n(data->ctrl, 6, held_control.begin());
      double zoh_difference = 0.0;
      for (int substep = 0; substep < kPhysicsSubstepsPerControl; ++substep) {
        for (int index = 0; index < 6; ++index)
          data->xfrc_applied[6 * base_body + index] = 0.0;
        const bool disturbed = options.disturbance_start_tick >= 0 &&
            tick >= options.disturbance_start_tick &&
            tick < options.disturbance_start_tick + options.disturbance_ticks;
        if (disturbed) {
          for (int axis = 0; axis < 3; ++axis) {
            data->xfrc_applied[6 * base_body + axis] = options.force[axis];
            data->xfrc_applied[6 * base_body + 3 + axis] = options.moment[axis];
          }
        }
        mj_step(model.get(), data.get());
        for (int actuator = 0; actuator < 6; ++actuator)
          zoh_difference = std::max(
              zoh_difference, std::abs(data->ctrl[actuator] - held_control[actuator]));
        const auto metrics = plantMetrics(model.get(), data.get());
        writePlantRow(
            plant, options, episode, tick, substep, data.get(), disturbed, metrics);
      }
      writeControlRow(control, options, episode, tick, pre_step_plant_time_s, state,
                      result, receipt_time_ns, core_step_ns, zoh_difference,
                      held_control, accepted);
      previous_source_time_ns = state.sample_time_ns;
    }
  }
}

}  // namespace

int main(int argc, char **argv) {
  try {
    run(parseOptions(argc, argv));
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "ERROR: " << error.what() << '\n';
    return 1;
  }
}
