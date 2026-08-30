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
    if (values.size() != header.size()) {
      throw std::runtime_error("CSV width mismatch");
    }
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
  state.sample_time_ns =
      static_cast<std::uint64_t>(std::stoull(row.at("source_ns")));
  for (int index = 0; index < 3; ++index) {
    state.base_position_n_m[index] = number(row, "base_p" + std::to_string(index));
    state.base_linear_velocity_n_m_s[index] =
        number(row, "base_v" + std::to_string(index));
    state.base_angular_velocity_n_rad_s[index] =
        number(row, "base_w" + std::to_string(index));
  }
  for (int index = 0; index < 4; ++index) {
    state.q_n_from_b[index] = number(row, "quat" + std::to_string(index));
  }
  for (int index = 0; index < 6; ++index) {
    state.joint_position_rad[index] = number(row, "q" + std::to_string(index));
    state.joint_velocity_rad_s[index] = number(row, "dq" + std::to_string(index));
  }
  state.contact_state[0] =
      static_cast<wheel_leg::ContactState>(std::stoi(row.at("contact_left")));
  state.contact_state[1] =
      static_cast<wheel_leg::ContactState>(std::stoi(row.at("contact_right")));
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

wheel_leg::WeightedWbcController::Result solveAtTick(
    const std::vector<Row> &rows, int target_tick,
    const Eigen::Vector2d &vertical_acceleration) {
  wheel_leg::WeightedWbcController controller(
      wheel_leg::WeightedWbcProfile::kPhase33ZetaManifold);
  wheel_leg::WeightedWbcController::Result result;
  bool found = false;
  for (const auto &row : rows) {
    const int tick = std::stoi(row.at("tick"));
    if (tick > target_tick) break;
    auto reference = referenceFrom(row);
    if (tick == target_tick) {
      reference.wheel_vertical_acceleration_m_s2 = vertical_acceleration;
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

void writeResult(int tick, const std::string &channel, double scale,
                 double sign,
                 const wheel_leg::WeightedWbcController::Result &result) {
  std::cout << tick << ',' << channel << ',' << scale << ',' << sign << ','
            << static_cast<int>(result.status) << ',' << result.hard_violation;
  for (const double value : result.wheel_position_b_z_m) {
    std::cout << ',' << value;
  }
  for (const double value : result.wheel_velocity_b_z_m_s) {
    std::cout << ',' << value;
  }
  for (const double value : result.wheel_vertical_acceleration_m_s2) {
    std::cout << ',' << value;
  }
  for (const double value : result.torque_nm) std::cout << ',' << value;
  for (const double value : result.realized_interaction_wrench_flu) {
    std::cout << ',' << value;
  }
  for (const double value : result.signed_interaction_slack_flu) {
    std::cout << ',' << value;
  }
  std::cout << '\n';
}

}  // namespace

int main(int argc, char **argv) {
  if (argc < 4) {
    std::cerr << "usage: phase33_wbc_zeta_sweep CONTROL.csv DELTA tick...\n";
    return 1;
  }
  try {
    const auto rows = readCsv(argv[1]);
    const double delta = std::stod(argv[2]);
    const std::array<std::pair<std::string, Eigen::Vector2d>, 4> channels{{
        {"left", Eigen::Vector2d(1.0, 0.0)},
        {"right", Eigen::Vector2d(0.0, 1.0)},
        {"common", Eigen::Vector2d(1.0, 1.0)},
        {"differential", Eigen::Vector2d(-1.0, 1.0)},
    }};
    std::cout << std::setprecision(17)
              << "tick,channel,step_scale,sign,status,hard_violation"
              << ",zeta0,zeta1,dzeta0,dzeta1,ddzeta0,ddzeta1";
    for (int index = 0; index < 6; ++index) std::cout << ",tau" << index;
    for (int index = 0; index < 12; ++index) std::cout << ",realized" << index;
    for (int index = 0; index < 12; ++index) std::cout << ",slack" << index;
    std::cout << '\n';
    for (int argument = 3; argument < argc; ++argument) {
      const int tick = std::stoi(argv[argument]);
      writeResult(tick, "baseline", 0.0, 0.0,
                  solveAtTick(rows, tick, Eigen::Vector2d::Zero()));
      for (const auto &[name, direction] : channels) {
        for (const double scale : {1.0, 0.5}) {
          for (const double sign : {-1.0, 1.0}) {
            writeResult(tick, name, scale, sign,
                        solveAtTick(rows, tick,
                                    sign * scale * delta * direction));
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
