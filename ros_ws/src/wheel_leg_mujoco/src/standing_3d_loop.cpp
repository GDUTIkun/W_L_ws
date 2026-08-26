#include <algorithm>
#include <array>
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

struct ModelDeleter { void operator()(mjModel *value) const { mj_deleteModel(value); } };
struct DataDeleter { void operator()(mjData *value) const { mj_deleteData(value); } };
using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;

struct Options {
  std::string model_path;
  std::string output_path;
  std::string scenario{"hold"};
  int episodes{1};
  int ticks{1000};
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
  wheel_leg::JointVector reference{
      -0.9719989158353384, 1.6393957458903228, 0.0,
      -0.9833909356455747, 1.6394010277077622, 0.0};
  wheel_leg::JointVector support{
      -0.04421779426760254, -3.1325438643315864, 0.0,
      0.041124284318536736, -3.2506637741279025, 0.0};
  wheel_leg::JointVector kp{8.0, 8.0, 0.0, 8.0, 8.0, 0.0};
  wheel_leg::JointVector kd{1.0, 1.0, 0.0, 1.0, 1.0, 0.0};
  wheel_leg::JointVector torque_limit{10.0, 10.0, 2.0, 10.0, 10.0, 2.0};
  std::array<double, 24> gain{};
  wheel_leg::JointVector roll_direction{
      0.027473966948114475, -0.7181968424472815, 0.0,
      -0.027056451311574754, 0.6947707716083865, 0.0};
  std::array<double, 8> safety{0.02, 0.02, 0.01, 0.03, 0.03, 0.05, 0.03, 10.0};
};

int requiredId(const mjModel *model, mjtObj type, const char *name) {
  const int id = mj_name2id(model, type, name);
  if (id < 0) throw std::runtime_error(std::string("Missing MuJoCo object: ") + name);
  return id;
}

int positive(const std::string &value, const char *name) {
  const int result = std::stoi(value);
  if (result <= 0) throw std::invalid_argument(std::string(name) + " must be positive");
  return result;
}

template <std::size_t Size>
std::array<double, Size> parseVector(const std::string &value, const char *name) {
  std::array<double, Size> result{};
  std::istringstream stream(value);
  std::string item;
  std::size_t index = 0;
  while (std::getline(stream, item, ',')) {
    if (index >= Size) throw std::invalid_argument(std::string(name) + " has too many values");
    result[index++] = std::stod(item);
  }
  if (index != Size || !std::all_of(result.begin(), result.end(),
      [](double number) { return std::isfinite(number); })) {
    throw std::invalid_argument(std::string(name) + " must contain finite values");
  }
  return result;
}

Options parseOptions(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (index + 1 >= argc) throw std::invalid_argument("Missing value for " + argument);
    const std::string value = argv[++index];
    if (argument == "--model") options.model_path = value;
    else if (argument == "--output") options.output_path = value;
    else if (argument == "--scenario") options.scenario = value;
    else if (argument == "--episodes") options.episodes = positive(value, "episodes");
    else if (argument == "--ticks") options.ticks = positive(value, "ticks");
    else if (argument == "--fault-tick") options.fault_tick = std::stoi(value);
    else if (argument == "--disturbance-start-tick") options.disturbance_start_tick = std::stoi(value);
    else if (argument == "--disturbance-ticks") options.disturbance_ticks = std::stoi(value);
    else if (argument == "--force") options.force = parseVector<3>(value, "force");
    else if (argument == "--moment") options.moment = parseVector<3>(value, "moment");
    else if (argument == "--initial-state") options.initial_state = parseVector<8>(value, "initial-state");
    else if (argument == "--leg-perturbation") options.leg_perturbation = parseVector<4>(value, "leg-perturbation");
    else if (argument == "--equilibrium") options.equilibrium = parseVector<9>(value, "equilibrium");
    else if (argument == "--reference") options.reference = parseVector<6>(value, "reference");
    else if (argument == "--support") options.support = parseVector<6>(value, "support");
    else if (argument == "--kp") options.kp = parseVector<6>(value, "kp");
    else if (argument == "--kd") options.kd = parseVector<6>(value, "kd");
    else if (argument == "--torque-limit") options.torque_limit = parseVector<6>(value, "torque-limit");
    else if (argument == "--gain") options.gain = parseVector<24>(value, "gain");
    else if (argument == "--roll-direction") options.roll_direction = parseVector<6>(value, "roll-direction");
    else if (argument == "--safety") options.safety = parseVector<8>(value, "safety");
    else throw std::invalid_argument("Unknown option: " + argument);
  }
  if (options.model_path.empty() || options.output_path.empty())
    throw std::invalid_argument("--model and --output are required");
  const std::array<std::string, 7> scenarios{
      "hold", "contact_loss_left", "contact_loss_right", "invalid",
      "nonmonotonic", "timing", "saturation"};
  if (std::find(scenarios.begin(), scenarios.end(), options.scenario) == scenarios.end())
    throw std::invalid_argument("Unknown scenario: " + options.scenario);
  return options;
}

template <typename Range>
void writeRange(std::ofstream &output, const Range &values) {
  for (const auto value : values) output << ',' << value;
}

void writeHeader(std::ofstream &output) {
  output << "scenario,episode,tick,time_s,status,safety_latched,command_accepted,"
            "contact_left,contact_right,x_m,vx_m_s,pitch_rad,wy_rad_s,roll_rad,"
            "wx_rad_s,yaw_rad,wz_rad_s,y_m,height_m,vy_m_s,vz_m_s,"
            "zoh_ctrl_max_difference,left_normal_load_n,right_normal_load_n,"
            "maximum_penetration_m,maximum_abs_rolling_slip_m_s,"
            "maximum_abs_lateral_slip_m_s,closure_residual_m";
  for (int i = 0; i < 6; ++i) output << ",q" << i;
  for (int i = 0; i < 6; ++i) output << ",dq" << i;
  for (int i = 0; i < 6; ++i) output << ",support" << i;
  for (int i = 0; i < 6; ++i) output << ",pd" << i;
  for (int i = 0; i < 6; ++i) output << ",raw" << i;
  for (int i = 0; i < 6; ++i) output << ",tau" << i;
  for (int i = 0; i < 6; ++i) output << ",ctrl" << i;
  for (int i = 0; i < 3; ++i) output << ",u" << i;
  output << '\n';
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
  for (int index = 0; index < data->ncon; ++index) {
    const auto &contact = data->contact[index];
    for (std::size_t side = 0; side < wheel_geom.size(); ++side) {
      const bool pair =
          (contact.geom1 == floor && contact.geom2 == wheel_geom[side]) ||
          (contact.geom2 == floor && contact.geom1 == wheel_geom[side]);
      if (!pair) continue;
      std::array<mjtNum, 6> local{};
      std::array<mjtNum, 3> world{};
      mj_contactForce(model, data, index, local.data());
      mju_mulMatTVec(world.data(), contact.frame, local.data(), 3, 3);
      if (contact.geom2 != wheel_geom[side])
        for (double &value : world) value = -value;
      result.normal_load_n[side] += world[2];
      result.maximum_penetration_m = std::max(
          result.maximum_penetration_m, std::max(0.0, -contact.dist));
    }
  }
  for (std::size_t side = 0; side < wheel_body.size(); ++side) {
    std::array<mjtNum, 6> velocity{};
    mj_objectVelocity(
        model, data, mjOBJ_BODY, wheel_body[side], velocity.data(), 0);
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
    double squared = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
      const double difference =
          data->site_xpos[3 * first + axis] - data->site_xpos[3 * second + axis];
      squared += difference * difference;
    }
    result.closure_residual_m = std::max(
        result.closure_residual_m, std::sqrt(squared));
  }
  return result;
}

bool bilateralContact(const mjModel *model, const mjData *data) {
  const int floor = requiredId(model, mjOBJ_GEOM, "floor");
  const std::array<int, 2> wheel{
      requiredId(model, mjOBJ_GEOM, "left_wheel_collision"),
      requiredId(model, mjOBJ_GEOM, "right_wheel_collision")};
  std::array<bool, 2> found{};
  for (int index = 0; index < data->ncon; ++index) {
    for (std::size_t side = 0; side < wheel.size(); ++side) {
      const auto &contact = data->contact[index];
      found[side] = found[side] ||
          ((contact.geom1 == floor && contact.geom2 == wheel[side]) ||
           (contact.geom2 == floor && contact.geom1 == wheel[side]));
    }
  }
  return found[0] && found[1];
}

void setInitialState(const mjModel *model, mjData *data, const Options &options) {
  data->qpos[0] = 0.0;
  data->qpos[1] = 0.0;
  data->qpos[2] = options.equilibrium[4];
  data->qpos[3] = 1.0;
  data->qpos[4] = data->qpos[5] = data->qpos[6] = 0.0;
  const std::array<const char *, 4> active{
      "right_hip_joint", "right_knee_joint", "left_hip_joint", "left_knee_joint"};
  for (std::size_t index = 0; index < active.size(); ++index) {
    const int joint = requiredId(model, mjOBJ_JOINT, active[index]);
    data->qpos[model->jnt_qposadr[joint]] = options.equilibrium[index];
  }
  const std::array<const char *, 4> passive{
      "right_connect1_joint", "right_connect2_joint",
      "left_connect1_joint", "left_connect2_joint"};
  for (std::size_t index = 0; index < passive.size(); ++index) {
    const int joint = requiredId(model, mjOBJ_JOINT, passive[index]);
    data->qpos[model->jnt_qposadr[joint]] = options.equilibrium[index + 5];
  }
  mj_forward(model, data);
  const int site = requiredId(model, mjOBJ_SITE, "base_control_frame");
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
    const int joint = requiredId(model, mjOBJ_JOINT, leg[index]);
    data->qpos[model->jnt_qposadr[joint]] -= options.leg_perturbation[index];
  }
  mj_forward(model, data);
  if (!bilateralContact(model, data))
    throw std::runtime_error("Initial state is outside bilateral contact mode");
}

int run(const Options &options) {
  if (std::filesystem::exists(options.output_path))
    throw std::runtime_error("Refusing to overwrite output file: " + options.output_path);
  char error[1024]{};
  ModelPtr model(mj_loadXML(options.model_path.c_str(), nullptr, error, sizeof(error)));
  if (!model) throw std::runtime_error(std::string("MuJoCo model load failed: ") + error);
  if (model->nq != 17 || model->nv != 16 || model->nu != 6 ||
      std::abs(model->opt.timestep - 0.002) > 1.0e-12)
    throw std::runtime_error("Full-3D standing compiled invariant failed");
  DataPtr data(mj_makeData(model.get()));
  wheel_leg_mujoco::AdapterConfig adapter_config;
  adapter_config.command_enabled = true;
  adapter_config.floating_base = true;
  wheel_leg_mujoco::Adapter adapter(model.get(), adapter_config);
  wheel_leg::ControllerConfig config;
  config.mode = wheel_leg::ControllerMode::kSimpleStanding3d;
  config.initial_reference.position_rad = options.reference;
  config.kp_nm_per_rad = options.kp;
  config.kd_nm_s_per_rad = options.kd;
  config.torque_limit_nm = options.torque_limit;
  config.simple_standing_3d.support_torque_nm = options.support;
  config.simple_standing_3d.roll_direction = options.roll_direction;
  for (std::size_t row = 0; row < 3; ++row)
    std::copy_n(options.gain.begin() + 8 * row, 8, config.simple_standing_3d.gain[row].begin());
  config.simple_standing_3d.maximum_abs_x_m = options.safety[0];
  config.simple_standing_3d.maximum_abs_y_m = options.safety[1];
  config.simple_standing_3d.maximum_height_error_m = options.safety[2];
  config.simple_standing_3d.maximum_abs_roll_rad = options.safety[3];
  config.simple_standing_3d.maximum_abs_pitch_rad = options.safety[4];
  config.simple_standing_3d.maximum_abs_yaw_rad = options.safety[5];
  config.simple_standing_3d.maximum_leg_error_rad = options.safety[6];
  config.simple_standing_3d.maximum_joint_velocity_rad_s = options.safety[7];
  wheel_leg::ControllerCore controller;
  if (!controller.configure(config)) throw std::runtime_error("3D controller configuration failed");
  const int base_body = requiredId(model.get(), mjOBJ_BODY, "base_body");
  std::ofstream output(options.output_path);
  if (!output) throw std::runtime_error("Cannot open output file: " + options.output_path);
  writeHeader(output);
  output << std::setprecision(17);
  constexpr std::uint64_t kControlPeriodNs = 10'000'000U;

  for (int episode = 0; episode < options.episodes; ++episode) {
    adapter.reset(data.get());
    controller.reset();
    setInitialState(model.get(), data.get(), options);
    std::uint64_t previous_source_time = 0;
    for (int tick = 0; tick < options.ticks; ++tick) {
      auto state = adapter.extractState(data.get());
      if (tick == options.fault_tick) {
        if (options.scenario == "contact_loss_left")
          state.contact_state[0] = wheel_leg::ContactState::kNoContact;
        else if (options.scenario == "contact_loss_right")
          state.contact_state[1] = wheel_leg::ContactState::kNoContact;
        else if (options.scenario == "invalid")
          state.q_n_from_b[0] = std::numeric_limits<double>::quiet_NaN();
        else if (options.scenario == "nonmonotonic") state.sample_time_ns = previous_source_time;
        else if (options.scenario == "timing") state.sample_time_ns += 1'000'000U;
        else if (options.scenario == "saturation") state.base_angular_velocity_n_rad_s[2] = 100.0;
      }
      const auto result = controller.step(state);
      const std::uint64_t receipt_time = static_cast<std::uint64_t>(tick) * kControlPeriodNs;
      const bool accepted = adapter.acceptCommand(
          result.command, receipt_time,
          wheel_leg_mujoco::Adapter::simulationTimeNs(data->time));
      adapter.writeControls(data.get(), receipt_time);
      std::array<double, 6> held{};
      std::copy_n(data->ctrl, 6, held.begin());
      double zoh_difference = 0.0;
      for (int step = 0; step < 5; ++step) {
        data->xfrc_applied[6 * base_body] = 0.0;
        data->xfrc_applied[6 * base_body + 1] = 0.0;
        data->xfrc_applied[6 * base_body + 2] = 0.0;
        data->xfrc_applied[6 * base_body + 3] = 0.0;
        data->xfrc_applied[6 * base_body + 4] = 0.0;
        data->xfrc_applied[6 * base_body + 5] = 0.0;
        const bool disturb = options.disturbance_start_tick >= 0 &&
            tick >= options.disturbance_start_tick &&
            tick < options.disturbance_start_tick + options.disturbance_ticks;
        if (disturb) {
          for (int axis = 0; axis < 3; ++axis) {
            data->xfrc_applied[6 * base_body + axis] = options.force[axis];
            data->xfrc_applied[6 * base_body + 3 + axis] = options.moment[axis];
          }
        }
        mj_step(model.get(), data.get());
        for (int actuator = 0; actuator < 6; ++actuator)
          zoh_difference = std::max(zoh_difference, std::abs(data->ctrl[actuator] - held[actuator]));
      }
      const auto plant = plantMetrics(model.get(), data.get());
      output << options.scenario << ',' << episode << ',' << tick << ',' << data->time
             << ',' << static_cast<int>(result.status) << ',' << result.safety_latched
             << ',' << accepted << ',' << static_cast<int>(state.contact_state[0])
             << ',' << static_cast<int>(state.contact_state[1]);
      writeRange(output, result.standing_state_3d);
      output << ',' << state.base_position_n_m[1] << ',' << state.base_position_n_m[2]
             << ',' << state.base_linear_velocity_n_m_s[1]
             << ',' << state.base_linear_velocity_n_m_s[2] << ',' << zoh_difference;
      writeRange(output, plant.normal_load_n);
      output << ',' << plant.maximum_penetration_m
             << ',' << plant.maximum_abs_rolling_slip_m_s
             << ',' << plant.maximum_abs_lateral_slip_m_s
             << ',' << plant.closure_residual_m;
      writeRange(output, state.joint_position_rad);
      writeRange(output, state.joint_velocity_rad_s);
      writeRange(output, result.tau_support_nm);
      writeRange(output, result.tau_pd_nm);
      writeRange(output, result.tau_raw_nm);
      writeRange(output, result.command.joint_torque_nm);
      writeRange(output, held);
      writeRange(output, result.virtual_input_3d);
      output << '\n';
      previous_source_time = state.sample_time_ns;
    }
  }
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  try { return run(parseOptions(argc, argv)); }
  catch (const std::exception &error) {
    std::cerr << "ERROR: " << error.what() << '\n';
    return 1;
  }
}
