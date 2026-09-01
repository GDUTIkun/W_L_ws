// Diagnostic-only Phase46 frozen-row reconstruction. Not linked into production.
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "wheel_leg_core/nominal_wbc_model.hpp"
#include "wheel_leg_core/weighted_wbc_problem.hpp"
#include "nominal_wbc_profile_data.hpp"

// This standalone diagnostic links only the model/problem translation units.
// The frozen runtime row has already passed the production state validator.
namespace wheel_leg {
ValidationError validateRobotState(const RobotState &, double) {
  return ValidationError::kNone;
}
}  // namespace wheel_leg

namespace {

using Row = std::unordered_map<std::string, std::string>;

std::vector<std::string> split(const std::string &line) {
  std::vector<std::string> values;
  std::stringstream stream(line);
  std::string value;
  while (std::getline(stream, value, ',')) values.push_back(value);
  return values;
}

Row readRow(const std::string &path) {
  std::ifstream input(path);
  std::string header_line, value_line;
  if (!std::getline(input, header_line) || !std::getline(input, value_line))
    throw std::runtime_error("expected header and one data row: " + path);
  const auto headers = split(header_line);
  const auto values = split(value_line);
  if (headers.size() != values.size())
    throw std::runtime_error("CSV column mismatch: " + path);
  Row row;
  for (std::size_t i = 0; i < headers.size(); ++i) row.emplace(headers[i], values[i]);
  return row;
}

double number(const Row &row, const std::string &key) {
  const auto found = row.find(key);
  if (found == row.end()) throw std::runtime_error("missing CSV field: " + key);
  return std::stod(found->second);
}

template <typename Derived>
void emit(const char *name, const Eigen::MatrixBase<Derived> &value) {
  for (Eigen::Index row = 0; row < value.rows(); ++row)
    for (Eigen::Index column = 0; column < value.cols(); ++column)
      std::cout << name << ',' << row << ',' << column << ',' << value(row, column) << '\n';
}

wheel_leg::RobotState stateFrom(const Row &row) {
  wheel_leg::RobotState state;
  for (int i = 0; i < 3; ++i) {
    state.base_position_n_m[i] = number(row, "base_p" + std::to_string(i));
    state.base_linear_velocity_n_m_s[i] = number(row, "base_v" + std::to_string(i));
    state.base_angular_velocity_n_rad_s[i] = number(row, "base_omega" + std::to_string(i));
  }
  for (int i = 0; i < 4; ++i) state.q_n_from_b[i] = number(row, "base_q" + std::to_string(i));
  for (int i = 0; i < 6; ++i) {
    state.joint_position_rad[i] = number(row, "q" + std::to_string(i));
    state.joint_velocity_rad_s[i] = number(row, "dq" + std::to_string(i));
  }
  state.contact_state = {wheel_leg::ContactState::kContact,
                         wheel_leg::ContactState::kContact};
  return state;
}

wheel_leg::WbcReference referenceFrom(const Row &row) {
  wheel_leg::WbcReference reference;
  reference.wheel_longitudinal_acceleration_m_s2 <<
      number(row, "desired_ddxi_left"), number(row, "desired_ddxi_right");
  reference.rolling_task_active = {number(row, "rolling_active_left") != 0.0,
                                   number(row, "rolling_active_right") != 0.0};
  reference.rolling_velocity_m_s << number(row, "rolling_velocity_left"),
      number(row, "rolling_velocity_right");
  reference.rolling_acceleration_m_s2 <<
      number(row, "desired_rolling_acceleration_left"),
      number(row, "desired_rolling_acceleration_right");
  reference.rolling_acceleration_bias_m_s2 << number(row, "rolling_bias_left"),
      number(row, "rolling_bias_right");
  for (int side = 0; side < 2; ++side)
    for (int column = 0; column < 12; ++column)
      reference.rolling_acceleration_map[side](0, column) = number(
          row, "rolling_map_" + std::to_string(side) + '_' + std::to_string(column));
  for (int i = 0; i < 12; ++i)
    reference.interaction_wrench_flu[i] = number(row, "requested_wrench" + std::to_string(i));
  if (row.find("r2_decision_row_rank") != row.end()) {
    reference.primitive_contact_active = true;
    reference.primitive_contact_row_count =
        static_cast<int>(number(row, "r2_decision_row_rank"));
    for (int r = 0; r < 12; ++r) {
      for (int c = 0; c < 12; ++c) {
        reference.primitive_contact_nudot(r, c) = number(
            row, "r2_decision_nudot_" + std::to_string(r) + '_' + std::to_string(c));
        reference.primitive_contact_wrench(r, c) = number(
            row, "r2_decision_wrench_" + std::to_string(r) + '_' + std::to_string(c));
      }
      reference.primitive_contact_rhs[r] =
          number(row, "r2_decision_rhs_" + std::to_string(r));
    }
  }
  const double common = 0.5 * (number(row, "delta2") + number(row, "delta3"));
  const double differential = 0.5 * (number(row, "delta3") - number(row, "delta2"));
  reference.hip_common_increment_limit_active =
      std::abs(common) > 0.0 && std::abs(differential) <= 1.0e-15;
  reference.nominal_hip_common_acceleration_rad_s2 = -0.009961062735978504;
  return reference;
}

}  // namespace

int main(int argc, char **argv) {
  try {
    if (argc < 2 || argc > 3)
      throw std::runtime_error(
          "usage: phase46_dump_qp_operators ROW.csv [point-realizable]");
    const Row row = readRow(argv[1]);
    const wheel_leg::NominalWbcModel::Result model =
        wheel_leg::NominalWbcModel{}.evaluate(stateFrom(row));
    if (!model.ok()) throw std::runtime_error("NominalWbcModel rejected frozen row");
    const auto reference = referenceFrom(row);
    const auto profile = argc == 3
        ? wheel_leg::WeightedWbcProfile::kPhase46PointRealizableRolling
        : wheel_leg::WeightedWbcProfile::kPhase46MujocoContactResponse;
    const auto problem = wheel_leg::WeightedWbcProblem{}.assemble(
        model, reference, profile);
    if (!problem.ok()) throw std::runtime_error("WeightedWbcProblem rejected frozen row");

    std::cout << std::setprecision(17);
    emit("mass", model.mass);
    emit("bias", model.bias);
    emit("actuation", model.actuation);
    for (int side = 0; side < 2; ++side) {
      const std::string suffix = std::to_string(side);
      emit(("wrench_map_" + suffix).c_str(), model.wrench_map[side]);
      emit(("contact_jacobian_" + suffix).c_str(), model.contact_jacobian[side]);
      emit(("contact_bias_" + suffix).c_str(), model.contact_bias[side]);
      emit(("contact_axis_" + suffix).c_str(), model.contact_axis[side]);
      emit(("point_force_wrench_projector_" + suffix).c_str(),
           model.point_force_wrench_projector[side]);
      emit(("interaction_acceleration_map_" + suffix).c_str(),
           model.interaction_acceleration_map[side]);
      emit(("interaction_contact_map_" + suffix).c_str(), model.interaction_contact_map[side]);
      emit(("interaction_bias_" + suffix).c_str(), model.interaction_bias[side]);
      emit(("wheel_longitudinal_map_" + suffix).c_str(),
           model.wheel_longitudinal_acceleration_map[side]);
      std::cout << "wheel_longitudinal_bias_" << suffix << ",0,0,"
                << model.wheel_longitudinal_acceleration_bias_m_s2[side] << '\n';
    }
    emit("h", problem.h);
    emit("g", problem.g);
    auto without_xi = reference;
    without_xi.wheel_longitudinal_acceleration_m_s2.setZero();
    const auto no_xi = wheel_leg::WeightedWbcProblem{}.assemble(
        model, without_xi,
        wheel_leg::WeightedWbcProfile::kPhase46HipCommonIncrementLimitedRolling);
    auto without_rolling = reference;
    without_rolling.rolling_acceleration_m_s2.setZero();
    const auto no_rolling = wheel_leg::WeightedWbcProblem{}.assemble(
        model, without_rolling,
        wheel_leg::WeightedWbcProfile::kPhase46HipCommonIncrementLimitedRolling);
    if (!no_xi.ok() || !no_rolling.ok())
      throw std::runtime_error("task-gradient counterfactual rejected");
    emit("g_xi_target", problem.g - no_xi.g);
    emit("g_rolling_target", problem.g - no_rolling.g);
    Eigen::Matrix<double, 42, 1> variable_scale;
    for (int i = 0; i < 42; ++i)
      variable_scale[i] = wheel_leg::phase21_profile::kVariableScale[i];
    emit("variable_scale", variable_scale);
    emit("a", problem.a);
    emit("lower", problem.lower);
    emit("upper", problem.upper);
    return 0;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
