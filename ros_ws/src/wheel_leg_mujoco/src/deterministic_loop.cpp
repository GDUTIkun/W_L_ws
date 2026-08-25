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
};

int parsePositive(const std::string &value, const char *name) {
  const int parsed = std::stoi(value);
  if (parsed <= 0) {
    throw std::invalid_argument(std::string(name) + " must be positive");
  }
  return parsed;
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
    } else {
      throw std::invalid_argument("Unknown option: " + argument);
    }
  }
  if (options.model_path.empty() || options.output_path.empty()) {
    throw std::invalid_argument("--model and --output are required");
  }
  if (options.scenario != "nominal" && options.scenario != "faults") {
    throw std::invalid_argument("scenario must be nominal or faults");
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
  if (!controller.configure(wheel_leg::ControllerConfig{})) {
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
  output << '\n' << std::setprecision(17);

  wheel_leg::TorqueCommand saved_old_command;
  bool have_saved_old_command = false;
  constexpr std::uint64_t kControlPeriodNs = 10'000'000U;
  constexpr std::uint64_t kPhysicsPeriodNs = 2'000'000U;

  for (int episode = 0; episode < options.episodes; ++episode) {
    adapter.reset(data.get());
    controller.reset();
    for (int tick = 0; tick < options.ticks; ++tick) {
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

      if (!result.accepted() || !allZero(result.command.joint_torque_nm)) {
        throw std::runtime_error("Phase 16 Core must accept finite state and output zero");
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
