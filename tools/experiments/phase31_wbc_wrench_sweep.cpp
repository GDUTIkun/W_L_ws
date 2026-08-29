#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "wheel_leg_core/weighted_wbc_controller.hpp"

namespace {

using Row = std::unordered_map<std::string, std::string>;

std::vector<std::string> split(const std::string &line) {
  std::vector<std::string> values;
  std::stringstream stream(line);
  for (std::string value; std::getline(stream, value, ',');) {
    values.push_back(value);
  }
  return values;
}

std::vector<Row> readCsv(const std::string &path) {
  std::ifstream stream(path);
  if (!stream) throw std::runtime_error("cannot open control CSV");
  std::string line;
  if (!std::getline(stream, line)) throw std::runtime_error("empty control CSV");
  const auto header = split(line);
  std::vector<Row> rows;
  while (std::getline(stream, line)) {
    const auto values = split(line);
    if (values.size() != header.size()) throw std::runtime_error("CSV width mismatch");
    Row row;
    for (std::size_t index = 0; index < header.size(); ++index) {
      row.emplace(header[index], values[index]);
    }
    rows.push_back(std::move(row));
  }
  return rows;
}

double number(const Row &row, const std::string &name) {
  return std::stod(row.at(name));
}

wheel_leg::RobotState stateFrom(const Row &row) {
  wheel_leg::RobotState state;
  state.sample_time_ns = static_cast<std::uint64_t>(std::stoull(row.at("source_ns")));
  for (int index = 0; index < 3; ++index) {
    state.base_position_n_m[index] = number(row, "base_p" + std::to_string(index));
    state.base_linear_velocity_n_m_s[index] = number(row, "base_v" + std::to_string(index));
    state.base_angular_velocity_n_rad_s[index] = number(row, "base_w" + std::to_string(index));
  }
  for (int index = 0; index < 4; ++index) {
    state.q_n_from_b[index] = number(row, "quat" + std::to_string(index));
  }
  for (int index = 0; index < 6; ++index) {
    state.joint_position_rad[index] = number(row, "q" + std::to_string(index));
    state.joint_velocity_rad_s[index] = number(row, "dq" + std::to_string(index));
  }
  state.contact_state[0] = static_cast<wheel_leg::ContactState>(std::stoi(row.at("contact_left")));
  state.contact_state[1] = static_cast<wheel_leg::ContactState>(std::stoi(row.at("contact_right")));
  return state;
}

wheel_leg::WbcReference referenceFrom(const Row &row) {
  wheel_leg::WbcReference reference;
  reference.base_x_acceleration_m_s2 = number(row, "reference0");
  reference.base_height_acceleration_m_s2 = number(row, "reference1");
  for (int index = 0; index < 3; ++index) {
    reference.orientation_acceleration_rad_s2[index] =
        number(row, "reference" + std::to_string(2 + index));
  }
  for (int index = 0; index < 4; ++index) {
    reference.leg_acceleration_rad_s2[index] =
        number(row, "reference" + std::to_string(5 + index));
  }
  for (int index = 0; index < 12; ++index) {
    reference.interaction_wrench_flu[index] =
        number(row, "reference" + std::to_string(9 + index));
  }
  return reference;
}

void perturb(wheel_leg::WbcReference &reference, const std::string &channel,
             double value) {
  if (channel == "common_fx") {
    reference.interaction_wrench_flu[0] += value;
    reference.interaction_wrench_flu[6] += value;
  } else if (channel == "differential_fx") {
    reference.interaction_wrench_flu[0] -= value;
    reference.interaction_wrench_flu[6] += value;
  } else if (channel == "common_ty") {
    reference.interaction_wrench_flu[4] += value;
    reference.interaction_wrench_flu[10] += value;
  } else if (channel == "differential_ty") {
    reference.interaction_wrench_flu[4] -= value;
    reference.interaction_wrench_flu[10] += value;
  } else {
    throw std::runtime_error("unknown channel");
  }
}

wheel_leg::WeightedWbcController::Result solveAtTick(
    const std::vector<Row> &rows, int target_tick,
    const std::string &channel = {}, double value = 0.0) {
  wheel_leg::WeightedWbcController controller(
      wheel_leg::WeightedWbcProfile::kPhase27Minimal);
  wheel_leg::WeightedWbcController::Result result;
  bool found = false;
  for (const auto &row : rows) {
    const int tick = std::stoi(row.at("tick"));
    if (tick > target_tick) break;
    auto reference = referenceFrom(row);
    if (tick == target_tick && !channel.empty()) {
      perturb(reference, channel, value);
    }
    result = controller.step(stateFrom(row), reference);
    if (tick == target_tick) {
      found = true;
      break;
    }
  }
  if (!found) throw std::runtime_error("tick not found");
  return result;
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 4) {
    std::cerr << "usage: phase31_wbc_wrench_sweep CONTROL.csv DELTA tick...\n";
    return 1;
  }
  try {
    const auto rows = readCsv(argv[1]);
    const double force_delta = std::stod(argv[2]);
    const std::array<std::string, 4> channels{
        "common_fx", "differential_fx", "common_ty", "differential_ty"};
    std::cout << std::setprecision(17)
              << "tick,channel,step_scale,sign,status";
    for (int index = 0; index < 6; ++index) std::cout << ",tau" << index;
    for (int index = 0; index < 12; ++index) std::cout << ",realized" << index;
    std::cout << '\n';
    for (int argument = 3; argument < argc; ++argument) {
      const int tick = std::stoi(argv[argument]);
      {
        const auto result = solveAtTick(rows, tick);
        std::cout << tick << ",baseline,0,0," << static_cast<int>(result.status);
        for (const double torque : result.torque_nm) std::cout << ',' << torque;
        for (const double value : result.realized_interaction_wrench_flu) {
          std::cout << ',' << value;
        }
        std::cout << '\n';
      }
      for (const auto &channel : channels) {
        const bool force_channel = channel.size() >= 2 &&
            channel.compare(channel.size() - 2, 2, "fx") == 0;
        const double channel_delta = force_channel ? force_delta : force_delta * 0.1;
        for (const double scale : {1.0, 0.5}) {
          for (const double sign : {-1.0, 1.0}) {
            const auto result = solveAtTick(
                rows, tick, channel, sign * scale * channel_delta);
            std::cout << tick << ',' << channel << ',' << scale << ',' << sign
                      << ',' << static_cast<int>(result.status);
            for (const double torque : result.torque_nm) std::cout << ',' << torque;
            for (const double value : result.realized_interaction_wrench_flu) {
              std::cout << ',' << value;
            }
            std::cout << '\n';
          }
        }
      }
    }
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
  return 0;
}
