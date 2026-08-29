#ifdef NDEBUG
#undef NDEBUG
#endif

#include <Eigen/Core>
#include <Eigen/LU>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

#include "wheel_leg_core/nominal_wbc_model.hpp"

#ifndef PHASE27_WHEEL_GOLDEN_PATH
#error "PHASE27_WHEEL_GOLDEN_PATH is required"
#endif

namespace {

template <typename Matrix>
double readMaximumError(const Matrix &actual, std::istream &input) {
  double maximum = 0.0;
  for (Eigen::Index row = 0; row < actual.rows(); ++row) {
    for (Eigen::Index column = 0; column < actual.cols(); ++column) {
      double expected = 0.0;
      input >> expected;
      maximum = std::max(maximum, std::abs(actual(row, column) - expected));
    }
  }
  return maximum;
}

}  // namespace

int main() {
  std::ifstream input(PHASE27_WHEEL_GOLDEN_PATH);
  std::string header;
  int sample_count = 0;
  input >> header >> sample_count;
  assert(input && header == "PHASE27_WHEEL_INTERACTION_GOLDEN_V1");
  assert(sample_count == 4);

  wheel_leg::NominalWbcModel model;
  double maximum_state_error = 0.0;
  double maximum_acceleration_error = 0.0;
  double maximum_contact_error = 0.0;
  double maximum_bias_error = 0.0;
  for (int sample = 0; sample < sample_count; ++sample) {
    std::string sample_id;
    wheel_leg::RobotState state;
    input >> sample_id;
    for (double &value : state.base_position_n_m) input >> value;
    for (double &value : state.q_n_from_b) input >> value;
    for (double &value : state.base_linear_velocity_n_m_s) input >> value;
    for (double &value : state.base_angular_velocity_n_rad_s) input >> value;
    for (double &value : state.joint_position_rad) input >> value;
    for (double &value : state.joint_velocity_rad_s) input >> value;
    state.contact_state = {wheel_leg::ContactState::kContact,
                           wheel_leg::ContactState::kContact};
    std::array<double, 2> expected_position{};
    std::array<double, 2> expected_velocity{};
    for (int side = 0; side < 2; ++side) {
      input >> expected_position[side] >> expected_velocity[side];
    }

    const auto result = model.evaluate(state);
    assert(result.ok());
    for (int side = 0; side < 2; ++side) {
      maximum_state_error = std::max({
          maximum_state_error,
          std::abs(result.wheel_position_b_x_m[side] -
                   expected_position[side]),
          std::abs(result.wheel_velocity_b_x_m_s[side] -
                   expected_velocity[side])});
      maximum_acceleration_error = std::max(
          maximum_acceleration_error,
          readMaximumError(result.interaction_acceleration_map[side], input));
      maximum_contact_error = std::max(
          maximum_contact_error,
          readMaximumError(result.interaction_contact_map[side], input));
      maximum_bias_error = std::max(
          maximum_bias_error,
          readMaximumError(result.interaction_bias[side], input));

      // Transport round-trip and virtual-work parity for deterministic
      // non-axis-aligned vectors. This also exercises every +/- component.
      Eigen::Matrix<double, 6, 1> contact_wrench;
      contact_wrench << 0.7, -0.4, 2.1, -0.08, 0.11, -0.05;
      const auto interaction_contact =
          result.interaction_contact_map[side] * contact_wrench;
      const auto round_trip = result.interaction_contact_map[side].inverse() *
          interaction_contact;
      assert((round_trip - contact_wrench).cwiseAbs().maxCoeff() <= 1.0e-12);
      Eigen::Matrix<double, 6, 1> wheel_twist;
      wheel_twist << -0.13, 0.09, 0.05, 0.17, -0.12, 0.08;
      const auto contact_twist =
          result.interaction_contact_map[side].transpose() * wheel_twist;
      assert(std::abs(interaction_contact.dot(wheel_twist) -
                      contact_wrench.dot(contact_twist)) <= 1.0e-12);

      Eigen::Matrix<double, 12, 1> acceleration;
      acceleration << 0.3, -0.2, 0.1, -0.4, 0.25, -0.15,
          0.7, -0.6, 0.5, -0.35, 0.45, -0.55;
      const auto realized =
          result.interaction_acceleration_map[side] * acceleration +
          interaction_contact + result.interaction_bias[side];
      Eigen::Matrix<double, 6, 1> requested;
      requested << 1.0, -0.5, 2.0, 0.1, -0.2, 0.3;
      const auto signed_slack = realized - requested;
      assert((realized - requested - signed_slack)
                 .cwiseAbs().maxCoeff() == 0.0);
    }
    assert(input);
    std::cout << sample_id << " state=" << maximum_state_error
              << " acceleration=" << maximum_acceleration_error
              << " contact=" << maximum_contact_error
              << " bias=" << maximum_bias_error << std::endl;
  }
  assert(maximum_state_error <= 2.0e-9);
  assert(maximum_acceleration_error <= 2.0e-8);
  assert(maximum_contact_error <= 2.0e-10);
  assert(maximum_bias_error <= 2.0e-5);
  return 0;
}
