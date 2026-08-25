#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
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
  std::string scenario{"nominal"};
  int episodes{2};
  int ticks{100};
  int physics_steps_per_control{5};
  bool enable_pd{true};
  bool enable_gravity{true};
  std::array<double, wheel_leg::kJointCount> reference{
      -1.3267204090965414, 2.2088002542738268, 0.0,
      -1.3267204090965414, 2.2088002542738268, 0.0};
  std::array<double, wheel_leg::kJointCount> reference_step{};
  std::array<double, wheel_leg::kJointCount> kp{
      12.0, 12.0, 0.3, 12.0, 12.0, 0.3};
  std::array<double, wheel_leg::kJointCount> kd{
      1.5, 1.5, 0.05, 1.5, 1.5, 0.05};
  std::array<double, wheel_leg::kJointCount> torque_limit{
      6.0, 6.0, 1.0, 6.0, 6.0, 1.0};
  std::array<double, wheel_leg::kJointCount> disturbance{};
  int reference_tick{-1};
  int disturbance_tick{-1};
  wheel_leg::GravityProfile gravity_profile{
      wheel_leg::currentNominalGravityProfile()};
};

int parsePositive(const std::string &value, const char *name) {
  const int parsed = std::stoi(value);
  if (parsed <= 0) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
  return parsed;
}

int parseInteger(const std::string &value, const char *name) {
  try {
    return std::stoi(value);
  } catch (const std::exception &) {
    throw std::invalid_argument(std::string(name) + " must be an integer");
  }
}

std::array<double, wheel_leg::kJointCount> parseJointVector(
    const std::string &value, const char *name) {
  std::array<double, wheel_leg::kJointCount> result{};
  std::istringstream stream(value);
  std::string item;
  std::size_t index = 0;
  while (std::getline(stream, item, ',')) {
    if (index >= result.size()) {
      throw std::invalid_argument(std::string(name) + " must contain six values");
    }
    result[index++] = std::stod(item);
  }
  if (index != result.size() || !std::all_of(
          result.begin(), result.end(),
          [](double number) { return std::isfinite(number); })) {
    throw std::invalid_argument(std::string(name) + " must contain six finite values");
  }
  return result;
}

std::array<double, 3> parseTriple(
    const std::string &value, const char *name) {
  std::array<double, 3> result{};
  std::istringstream stream(value);
  std::string item;
  std::size_t index = 0;
  while (std::getline(stream, item, ',')) {
    if (index >= result.size()) {
      throw std::invalid_argument(std::string(name) + " must contain three values");
    }
    result[index++] = std::stod(item);
  }
  if (index != result.size() || !std::all_of(
          result.begin(), result.end(),
          [](double number) { return std::isfinite(number); })) {
    throw std::invalid_argument(std::string(name) + " must contain three finite values");
  }
  return result;
}

void setHarmonicCoefficients(
    std::array<wheel_leg::GravityHarmonic, 3> &harmonics,
    const std::array<double, 3> &values, bool sine) {
  for (std::size_t index = 0; index < harmonics.size(); ++index) {
    if (sine) {
      harmonics[index].sin_torque_nm = values[index];
    } else {
      harmonics[index].cos_torque_nm = values[index];
    }
  }
}

Options parseOptions(int argc, char **argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--version") {
      std::cout << "wheel_leg_mujoco_deterministic_loop schema=1 mujoco="
                << mj_versionString() << '\n';
      std::exit(0);
    }
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
    } else if (argument == "--physics-steps-per-control") {
      options.physics_steps_per_control =
          parsePositive(value, "physics-steps-per-control");
    } else if (argument == "--enable-pd") {
      options.enable_pd = parseInteger(value, "enable-pd") != 0;
    } else if (argument == "--enable-gravity") {
      options.enable_gravity = parseInteger(value, "enable-gravity") != 0;
    } else if (argument == "--reference") {
      options.reference = parseJointVector(value, "reference");
    } else if (argument == "--reference-step") {
      options.reference_step = parseJointVector(value, "reference-step");
    } else if (argument == "--kp") {
      options.kp = parseJointVector(value, "kp");
    } else if (argument == "--kd") {
      options.kd = parseJointVector(value, "kd");
    } else if (argument == "--torque-limit") {
      options.torque_limit = parseJointVector(value, "torque-limit");
    } else if (argument == "--disturbance") {
      options.disturbance = parseJointVector(value, "disturbance");
    } else if (argument == "--reference-tick") {
      options.reference_tick = parseInteger(value, "reference-tick");
    } else if (argument == "--disturbance-tick") {
      options.disturbance_tick = parseInteger(value, "disturbance-tick");
    } else if (argument == "--gravity-offset") {
      const auto offsets = parseJointVector(value, "gravity-offset");
      std::copy_n(
          offsets.begin(), 3,
          options.gravity_profile.left.canonical_offset_rad.begin());
      std::copy_n(
          offsets.begin() + 3, 3,
          options.gravity_profile.right.canonical_offset_rad.begin());
    } else if (argument == "--gravity-left-sin") {
      setHarmonicCoefficients(
          options.gravity_profile.left.harmonics,
          parseTriple(value, "gravity-left-sin"), true);
    } else if (argument == "--gravity-left-cos") {
      setHarmonicCoefficients(
          options.gravity_profile.left.harmonics,
          parseTriple(value, "gravity-left-cos"), false);
    } else if (argument == "--gravity-right-sin") {
      setHarmonicCoefficients(
          options.gravity_profile.right.harmonics,
          parseTriple(value, "gravity-right-sin"), true);
    } else if (argument == "--gravity-right-cos") {
      setHarmonicCoefficients(
          options.gravity_profile.right.harmonics,
          parseTriple(value, "gravity-right-cos"), false);
    } else {
      throw std::invalid_argument("Unknown option: " + argument);
    }
  }
  if (options.model_path.empty() || options.output_path.empty()) {
    throw std::invalid_argument("--model and --output are required");
  }
  if (options.scenario != "nominal" && options.scenario != "faults" &&
      options.scenario != "control") {
    throw std::invalid_argument("scenario must be nominal, faults, or control");
  }
  if (options.scenario == "faults" &&
      (options.episodes < 2 || options.ticks < 20)) {
    throw std::invalid_argument("faults requires at least 2 episodes and 20 ticks");
  }
  return options;
}

void writeIndexedHeader(std::ofstream &stream, const char *prefix, int count) {
  for (int index = 0; index < count; ++index) {
    stream << ',' << prefix << index;
  }
}

template <typename Range>
void writeRange(std::ofstream &stream, const Range &values) {
  for (const auto value : values) {
    stream << ',' << value;
  }
}

bool allZero(const std::array<double, wheel_leg::kJointCount> &values) {
  return std::all_of(
      values.begin(), values.end(), [](double value) { return value == 0.0; });
}

int run(const Options &options) {
  if (std::filesystem::exists(options.output_path)) {
    throw std::runtime_error("Refusing to overwrite output file: " +
                             options.output_path);
  }

  char error[1024]{};
  ModelPtr model(
      mj_loadXML(options.model_path.c_str(), nullptr, error, sizeof(error)));
  if (!model) {
    throw std::runtime_error(std::string("MuJoCo model load failed: ") + error);
  }
  if (std::abs(model->opt.timestep - 0.002) > 1.0e-12 ||
      options.physics_steps_per_control != 5) {
    throw std::runtime_error("Phase 16 requires 2 ms physics and 5-step ZOH");
  }

  DataPtr data(mj_makeData(model.get()));
  if (!data) {
    throw std::runtime_error("MuJoCo data allocation failed");
  }
  wheel_leg_mujoco::AdapterConfig adapter_config;
  adapter_config.command_enabled = true;
  wheel_leg_mujoco::Adapter adapter(model.get(), adapter_config);
  wheel_leg::ControllerCore controller;
  wheel_leg::ControllerConfig controller_config;
  if (options.scenario == "control") {
    controller_config.mode = wheel_leg::ControllerMode::kJointPdGravity;
    controller_config.enable_pd = options.enable_pd;
    controller_config.enable_gravity = options.enable_gravity;
    controller_config.initial_reference.position_rad = options.reference;
    controller_config.kp_nm_per_rad = options.kp;
    controller_config.kd_nm_s_per_rad = options.kd;
    controller_config.torque_limit_nm = options.torque_limit;
    controller_config.gravity_profile = options.gravity_profile;
  }
  if (!controller.configure(controller_config)) {
    throw std::runtime_error("Controller configuration failed");
  }

  std::ofstream output(options.output_path);
  if (!output) {
    throw std::runtime_error("Cannot open output file: " + options.output_path);
  }
  output << "scenario,episode,tick,physics_begin,physics_end,source_time_ns,"
            "receipt_time_ns,core_status,probe_status,dt_s,command_attempted,"
            "command_accepted,probe_command_attempted,probe_command_accepted,"
            "fault_event,injected_torque_nm,zoh_ctrl_max_difference,contact_left,"
            "contact_right";
  writeIndexedHeader(output, "base_position_", 3);
  writeIndexedHeader(output, "base_quaternion_", 4);
  writeIndexedHeader(output, "base_linear_velocity_", 3);
  writeIndexedHeader(output, "base_angular_velocity_", 3);
  writeIndexedHeader(output, "q_", wheel_leg::kJointCount);
  writeIndexedHeader(output, "dq_", wheel_leg::kJointCount);
  writeIndexedHeader(output, "tau_", wheel_leg::kJointCount);
  writeIndexedHeader(output, "ctrl_", wheel_leg::kJointCount);
  writeIndexedHeader(output, "reference_", wheel_leg::kJointCount);
  writeIndexedHeader(output, "tau_pd_", wheel_leg::kJointCount);
  writeIndexedHeader(output, "tau_gravity_", wheel_leg::kJointCount);
  writeIndexedHeader(output, "tau_raw_", wheel_leg::kJointCount);
  writeIndexedHeader(output, "saturated_", wheel_leg::kJointCount);
  writeIndexedHeader(output, "disturbance_", wheel_leg::kJointCount);
  output << '\n' << std::setprecision(17);

  wheel_leg::TorqueCommand saved_old_command;
  bool have_saved_old_command = false;
  constexpr std::uint64_t kControlPeriodNs = 10'000'000U;
  constexpr std::uint64_t kPhysicsPeriodNs = 2'000'000U;
  std::array<int, wheel_leg::kJointCount> driven_dofs{};
  for (std::size_t joint = 0; joint < wheel_leg::kJointCount; ++joint) {
    const int joint_id = model->actuator_trnid[2 * static_cast<int>(joint)];
    driven_dofs[joint] = model->jnt_dofadr[joint_id];
  }

  for (int episode = 0; episode < options.episodes; ++episode) {
    adapter.reset(data.get());
    controller.reset();
    for (int tick = 0; tick < options.ticks; ++tick) {
      wheel_leg::JointReference active_reference;
      active_reference.position_rad = options.reference;
      if (options.scenario == "control" && options.reference_tick >= 0 &&
          tick >= options.reference_tick) {
        for (std::size_t joint = 0; joint < wheel_leg::kJointCount; ++joint) {
          active_reference.position_rad[joint] += options.reference_step[joint];
        }
      }
      if (options.scenario == "control" &&
          !controller.setReference(active_reference)) {
        throw std::runtime_error("Controller reference update failed");
      }
      const std::uint64_t receipt_time_ns =
          static_cast<std::uint64_t>(tick) * kControlPeriodNs;
      const auto state = adapter.extractState(data.get());
      const auto result = controller.step(state);
      int probe_status = -1;
      bool command_attempted = true;
      bool command_accepted = false;
      bool probe_command_attempted = false;
      bool probe_command_accepted = false;
      double injected_torque_nm = 0.0;
      std::string fault_event = "none";

      if (options.scenario == "faults" && episode == 0 && tick == 1) {
        probe_status = static_cast<int>(controller.step(state).status);
        fault_event = "duplicate_state";
      }

      wheel_leg::TorqueCommand adapter_command = result.command;
      if (options.scenario == "faults" && episode == 0 && tick == 2) {
        ++adapter_command.source_sample_time_ns;
        fault_event = "future_command";
      } else if (options.scenario == "faults" && episode == 0 && tick == 6) {
        adapter_command.source_sample_time_ns = 0;
        fault_event = "stale_command";
      } else if (options.scenario == "faults" && episode == 0 && tick == 7) {
        adapter_command.joint_torque_nm[0] = 1.0;
        injected_torque_nm = 1.0;
        saved_old_command = adapter_command;
        have_saved_old_command = true;
        fault_event = "timeout_seed";
      } else if (options.scenario == "faults" && episode == 0 && tick == 8) {
        command_attempted = false;
        fault_event = "receipt_timeout";
      } else if (options.scenario == "faults" && episode == 0 && tick == 9) {
        fault_event = "recovery";
      }

      if (options.scenario == "faults" && episode == 1 && tick == 0) {
        if (!have_saved_old_command) {
          throw std::runtime_error("Missing saved pre-reset command");
        }
        probe_command_attempted = true;
        probe_command_accepted = adapter.acceptCommand(
            saved_old_command, receipt_time_ns, state.sample_time_ns);
        fault_event = "reset_old_recovery";
      }

      if (command_attempted) {
        command_accepted = adapter.acceptCommand(
            adapter_command, receipt_time_ns, state.sample_time_ns);
      }

      std::array<double, wheel_leg::kJointCount> interval_ctrl{};
      double zoh_ctrl_max_difference = 0.0;
      for (int physics_step = 0;
           physics_step < options.physics_steps_per_control; ++physics_step) {
        std::uint64_t physics_receipt_time_ns =
            receipt_time_ns +
            static_cast<std::uint64_t>(physics_step) * kPhysicsPeriodNs;
        if (options.scenario == "faults" && episode == 0 && tick == 8) {
          physics_receipt_time_ns += 100'000'001U;
        }
        adapter.writeControls(data.get(), physics_receipt_time_ns);
        std::fill(data->qfrc_applied, data->qfrc_applied + model->nv, 0.0);
        if (options.scenario == "control" &&
            tick == options.disturbance_tick) {
          for (std::size_t joint = 0; joint < wheel_leg::kJointCount; ++joint) {
            data->qfrc_applied[driven_dofs[joint]] =
                -options.disturbance[joint];
          }
        }
        if (physics_step == 0) {
          std::copy_n(data->ctrl, wheel_leg::kJointCount, interval_ctrl.begin());
        } else {
          for (std::size_t joint = 0; joint < wheel_leg::kJointCount; ++joint) {
            zoh_ctrl_max_difference = std::max(
                zoh_ctrl_max_difference,
                std::abs(data->ctrl[joint] - interval_ctrl[joint]));
          }
        }
        mj_step(model.get(), data.get());
      }

      if (!result.accepted() ||
          (options.scenario != "control" &&
           !allZero(result.command.joint_torque_nm))) {
        throw std::runtime_error("Controller output invariant failed");
      }
      output << options.scenario << ',' << episode << ',' << tick << ','
             << tick * options.physics_steps_per_control << ','
             << (tick + 1) * options.physics_steps_per_control << ','
             << state.sample_time_ns << ',' << receipt_time_ns << ','
             << static_cast<int>(result.status) << ',' << probe_status << ','
             << result.dt_s << ',' << static_cast<int>(command_attempted) << ','
             << static_cast<int>(command_accepted) << ','
             << static_cast<int>(probe_command_attempted) << ','
             << static_cast<int>(probe_command_accepted) << ',' << fault_event
             << ',' << injected_torque_nm << ',' << zoh_ctrl_max_difference
             << ',' << static_cast<int>(state.contact_state[0]) << ','
             << static_cast<int>(state.contact_state[1]);
      writeRange(output, state.base_position_n_m);
      writeRange(output, state.q_n_from_b);
      writeRange(output, state.base_linear_velocity_n_m_s);
      writeRange(output, state.base_angular_velocity_n_rad_s);
      writeRange(output, state.joint_position_rad);
      writeRange(output, state.joint_velocity_rad_s);
      writeRange(output, result.command.joint_torque_nm);
      writeRange(output, interval_ctrl);
      writeRange(output, active_reference.position_rad);
      writeRange(output, result.tau_pd_nm);
      writeRange(output, result.tau_gravity_nm);
      writeRange(output, result.tau_raw_nm);
      writeRange(output, result.saturated);
      const auto applied_disturbance =
          (tick == options.disturbance_tick) ? options.disturbance
                                             : decltype(options.disturbance){};
      writeRange(output, applied_disturbance);
      output << '\n';
    }
  }
  return 0;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    return run(parseOptions(argc, argv));
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
