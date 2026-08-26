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

#include "mujoco/mujoco.h"
#include "wheel_leg_core/controller_core.hpp"
#include "wheel_leg_mujoco/adapter.hpp"

namespace {

struct ModelDeleter {
  void operator()(mjModel *model) const { mj_deleteModel(model); }
};

struct DataDeleter {
  void operator()(mjData *data) const { mj_deleteData(data); }
};

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
  double force_x_n{0.0};
  double pitch_moment_nm{0.0};
  std::array<double, 4> initial_state{};
  std::array<double, 4> leg_perturbation{};
  std::array<double, 9> equilibrium{
      -0.3431775504191725, 0.5696975530916328,
      -0.3622567626778332, 0.5683402180546114,
      -0.2522596538884179, -0.5727296386244646,
      -0.5731565367836947, 0.5714520442495344,
      -0.5718682242479592};
  wheel_leg::JointVector reference{
      -0.9644636467095591, 1.6404600368321115, 0.0,
      -0.9835428589682198, 1.6391027017950901, 0.0};
  wheel_leg::JointVector support{
      -0.1558228810012853, -1.9558769180289424, 0.0,
      0.15272937105260695, -4.428369830750493, 0.0};
  wheel_leg::JointVector kp{8.0, 8.0, 0.0, 8.0, 8.0, 0.0};
  wheel_leg::JointVector kd{1.0, 1.0, 0.0, 1.0, 1.0, 0.0};
  wheel_leg::JointVector torque_limit{10.0, 10.0, 2.0, 10.0, 10.0, 2.0};
  std::array<double, 4> gain{
      2.939716938854372, 5.562926010255381,
      39.49917235500377, 0.6296413262862853};
  std::array<double, 5> safety{0.03, 0.02, 0.01, 0.03, 10.0};
};

int requiredId(const mjModel *model, mjtObj type, const char *name) {
  const int id = mj_name2id(model, type, name);
  if (id < 0) {
    throw std::runtime_error(std::string("Missing MuJoCo object: ") + name);
  }
  return id;
}

int parsePositive(const std::string &value, const char *name) {
  const int parsed = std::stoi(value);
  if (parsed <= 0) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
  return parsed;
}

template <std::size_t Size>
std::array<double, Size> parseVector(
    const std::string &value, const char *name) {
  std::array<double, Size> result{};
  std::istringstream stream(value);
  std::string item;
  std::size_t index = 0;
  while (std::getline(stream, item, ',')) {
    if (index >= Size) {
      throw std::invalid_argument(std::string(name) + " has too many values");
    }
    result[index++] = std::stod(item);
  }
  if (index != Size || !std::all_of(
          result.begin(), result.end(),
          [](double number) { return std::isfinite(number); })) {
    throw std::invalid_argument(
        std::string(name) + " must contain finite values");
  }
  return result;
}

Options parseOptions(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (index + 1 >= argc) {
      throw std::invalid_argument("Missing value for " + argument);
    }
    const std::string value = argv[++index];
    if (argument == "--model") {
      options.model_path = value;
    } else if (argument == "--output") {
      options.output_path = value;
    } else if (argument == "--scenario") {
      options.scenario = value;
    } else if (argument == "--episodes") {
      options.episodes = parsePositive(value, "episodes");
    } else if (argument == "--ticks") {
      options.ticks = parsePositive(value, "ticks");
    } else if (argument == "--fault-tick") {
      options.fault_tick = std::stoi(value);
    } else if (argument == "--disturbance-start-tick") {
      options.disturbance_start_tick = std::stoi(value);
    } else if (argument == "--disturbance-ticks") {
      options.disturbance_ticks = std::stoi(value);
    } else if (argument == "--force-x") {
      options.force_x_n = std::stod(value);
    } else if (argument == "--pitch-moment") {
      options.pitch_moment_nm = std::stod(value);
    } else if (argument == "--initial-state") {
      options.initial_state = parseVector<4>(value, "initial-state");
    } else if (argument == "--leg-perturbation") {
      options.leg_perturbation = parseVector<4>(value, "leg-perturbation");
    } else if (argument == "--equilibrium") {
      options.equilibrium = parseVector<9>(value, "equilibrium");
    } else if (argument == "--reference") {
      options.reference = parseVector<6>(value, "reference");
    } else if (argument == "--support") {
      options.support = parseVector<6>(value, "support");
    } else if (argument == "--kp") {
      options.kp = parseVector<6>(value, "kp");
    } else if (argument == "--kd") {
      options.kd = parseVector<6>(value, "kd");
    } else if (argument == "--torque-limit") {
      options.torque_limit = parseVector<6>(value, "torque-limit");
    } else if (argument == "--gain") {
      options.gain = parseVector<4>(value, "gain");
    } else if (argument == "--safety") {
      options.safety = parseVector<5>(value, "safety");
    } else {
      throw std::invalid_argument("Unknown option: " + argument);
    }
  }
  if (options.model_path.empty() || options.output_path.empty()) {
    throw std::invalid_argument("--model and --output are required");
  }
  const std::array<std::string, 5> scenarios{
      "hold", "contact_loss", "invalid", "nonmonotonic", "saturation"};
  if (std::find(scenarios.begin(), scenarios.end(), options.scenario) ==
      scenarios.end()) {
    throw std::invalid_argument("Unknown scenario: " + options.scenario);
  }
  return options;
}

template <typename Range>
void writeRange(std::ofstream &stream, const Range &values) {
  for (const auto value : values) {
    stream << ',' << value;
  }
}

void writeHeader(std::ofstream &output) {
  output << "scenario,episode,tick,time_s,status,safety_latched,command_accepted,"
            "contact_left,contact_right,x,dx,pitch,dtheta,height,zoh_ctrl_max_difference";
  for (int index = 0; index < 6; ++index) output << ",q" << index;
  for (int index = 0; index < 6; ++index) output << ",dq" << index;
  for (int index = 0; index < 6; ++index) output << ",tau" << index;
  for (int index = 0; index < 6; ++index) output << ",ctrl" << index;
  output << '\n';
}

bool bilateralWheelContact(const mjModel *model, const mjData *data) {
  const int floor = requiredId(model, mjOBJ_GEOM, "floor");
  const std::array<int, 2> wheels{
      requiredId(model, mjOBJ_GEOM, "left_wheel_collision"),
      requiredId(model, mjOBJ_GEOM, "right_wheel_collision")};
  std::array<bool, 2> present{};
  for (int contact_index = 0; contact_index < data->ncon; ++contact_index) {
    const auto &contact = data->contact[contact_index];
    for (std::size_t side = 0; side < wheels.size(); ++side) {
      present[side] = present[side] ||
          ((contact.geom1 == floor && contact.geom2 == wheels[side]) ||
           (contact.geom2 == floor && contact.geom1 == wheels[side]));
    }
  }
  return present[0] && present[1];
}

void setInitialState(
    const mjModel *model, mjData *data, const Options &options) {
  const std::array<const char *, 9> joints{
      "right_hip_joint", "right_knee_joint", "left_hip_joint",
      "left_knee_joint", "base_z_joint", "right_connect1_joint",
      "right_connect2_joint", "left_connect1_joint", "left_connect2_joint"};
  for (std::size_t index = 0; index < joints.size(); ++index) {
    const int joint = requiredId(model, mjOBJ_JOINT, joints[index]);
    data->qpos[model->jnt_qposadr[joint]] = options.equilibrium[index];
  }
  const int base_x = requiredId(model, mjOBJ_JOINT, "base_x_joint");
  const int base_pitch = requiredId(model, mjOBJ_JOINT, "base_pitch_joint");
  data->qpos[model->jnt_qposadr[base_x]] += options.initial_state[0];
  data->qvel[model->jnt_dofadr[base_x]] = options.initial_state[1];
  data->qpos[model->jnt_qposadr[base_pitch]] += options.initial_state[2];
  data->qvel[model->jnt_dofadr[base_pitch]] = options.initial_state[3];
  const std::array<const char *, 4> active_joints{
      "left_hip_joint", "left_knee_joint",
      "right_hip_joint", "right_knee_joint"};
  for (std::size_t index = 0; index < active_joints.size(); ++index) {
    const int joint = requiredId(model, mjOBJ_JOINT, active_joints[index]);
    data->qpos[model->jnt_qposadr[joint]] -= options.leg_perturbation[index];
  }
  mj_forward(model, data);
  const int base_z = requiredId(model, mjOBJ_JOINT, "base_z_joint");
  for (int step = 0; !bilateralWheelContact(model, data) && step < 500; ++step) {
    data->qpos[model->jnt_qposadr[base_z]] -= 1.0e-5;
    mj_forward(model, data);
  }
  if (!bilateralWheelContact(model, data)) {
    throw std::runtime_error("Could not project reset onto bilateral wheel contact");
  }
}

int run(const Options &options) {
  if (std::filesystem::exists(options.output_path)) {
    throw std::runtime_error("Refusing to overwrite output file: " +
                             options.output_path);
  }
  char error[1024]{};
  ModelPtr model(mj_loadXML(
      options.model_path.c_str(), nullptr, error, sizeof(error)));
  if (!model) {
    throw std::runtime_error(std::string("MuJoCo model load failed: ") + error);
  }
  if (std::abs(model->opt.timestep - 0.002) > 1.0e-12) {
    throw std::runtime_error("Planar standing requires 2 ms physics");
  }
  DataPtr data(mj_makeData(model.get()));
  wheel_leg_mujoco::AdapterConfig adapter_config;
  adapter_config.command_enabled = true;
  adapter_config.floating_base = true;
  wheel_leg_mujoco::Adapter adapter(model.get(), adapter_config);
  wheel_leg::ControllerConfig controller_config;
  controller_config.mode = wheel_leg::ControllerMode::kSimpleStanding;
  controller_config.initial_reference.position_rad = options.reference;
  controller_config.kp_nm_per_rad = options.kp;
  controller_config.kd_nm_s_per_rad = options.kd;
  controller_config.torque_limit_nm = options.torque_limit;
  controller_config.simple_standing.support_torque_nm = options.support;
  controller_config.simple_standing.gain = options.gain;
  controller_config.simple_standing.maximum_abs_pitch_rad = options.safety[0];
  controller_config.simple_standing.maximum_abs_x_m = options.safety[1];
  controller_config.simple_standing.maximum_height_error_m = options.safety[2];
  controller_config.simple_standing.maximum_leg_error_rad = options.safety[3];
  controller_config.simple_standing.maximum_joint_velocity_rad_s = options.safety[4];
  wheel_leg::ControllerCore controller;
  if (!controller.configure(controller_config)) {
    throw std::runtime_error("Standing controller configuration failed");
  }
  const int base_body = requiredId(model.get(), mjOBJ_BODY, "base_body");
  std::ofstream output(options.output_path);
  if (!output) {
    throw std::runtime_error("Cannot open output file: " + options.output_path);
  }
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
        if (options.scenario == "contact_loss") {
          state.contact_state[0] = wheel_leg::ContactState::kNoContact;
        } else if (options.scenario == "invalid") {
          state.q_n_from_b[0] = std::numeric_limits<double>::quiet_NaN();
        } else if (options.scenario == "nonmonotonic") {
          state.sample_time_ns = previous_source_time;
        } else if (options.scenario == "saturation") {
          state.base_linear_velocity_n_m_s[0] = 1.0;
        }
      }
      const auto result = controller.step(state);
      const std::uint64_t receipt_time =
          static_cast<std::uint64_t>(tick) * kControlPeriodNs;
      const bool accepted = adapter.acceptCommand(
          result.command, receipt_time,
          wheel_leg_mujoco::Adapter::simulationTimeNs(data->time));
      adapter.writeControls(data.get(), receipt_time);
      std::array<double, 6> held_controls{};
      std::copy_n(data->ctrl, 6, held_controls.begin());
      double zoh_difference = 0.0;
      for (int physics_step = 0; physics_step < 5; ++physics_step) {
        const bool disturbance_active =
            options.disturbance_start_tick >= 0 &&
            tick >= options.disturbance_start_tick &&
            tick < options.disturbance_start_tick + options.disturbance_ticks;
        data->xfrc_applied[6 * base_body] =
            disturbance_active ? options.force_x_n : 0.0;
        data->xfrc_applied[6 * base_body + 4] =
            disturbance_active ? options.pitch_moment_nm : 0.0;
        mj_step(model.get(), data.get());
        for (int actuator = 0; actuator < 6; ++actuator) {
          zoh_difference = std::max(
              zoh_difference,
              std::abs(data->ctrl[actuator] - held_controls[actuator]));
        }
      }
      output << options.scenario << ',' << episode << ',' << tick << ','
             << data->time << ',' << static_cast<int>(result.status) << ','
             << result.safety_latched << ',' << accepted << ','
             << static_cast<int>(state.contact_state[0]) << ','
             << static_cast<int>(state.contact_state[1]);
      writeRange(output, result.standing_state);
      output << ',' << state.base_position_n_m[2] << ',' << zoh_difference;
      writeRange(output, state.joint_position_rad);
      writeRange(output, state.joint_velocity_rad_s);
      writeRange(output, result.command.joint_torque_nm);
      std::array<double, 6> controls{};
      std::copy_n(data->ctrl, 6, controls.begin());
      writeRange(output, controls);
      output << '\n';
      previous_source_time = state.sample_time_ns;
    }
  }
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    return run(parseOptions(argc, argv));
  } catch (const std::exception &error) {
    std::cerr << "ERROR: " << error.what() << '\n';
    return 1;
  }
}
