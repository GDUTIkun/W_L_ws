#ifdef NDEBUG
#undef NDEBUG
#endif

#include <Eigen/Core>
#include <Eigen/LU>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

#include "wheel_leg_core/nominal_wbc_model.hpp"

#ifndef WBC_MODEL_GOLDEN_PATH
#error "WBC_MODEL_GOLDEN_PATH is required"
#endif

namespace {

double maximumError(const Eigen::Ref<const Eigen::MatrixXd> &actual,
                    std::istream &input) {
  double error = 0.0;
  for (Eigen::Index row = 0; row < actual.rows(); ++row) {
    for (Eigen::Index column = 0; column < actual.cols(); ++column) {
      double expected = 0.0;
      input >> expected;
      error = std::max(error, std::abs(actual(row, column) - expected));
    }
  }
  return error;
}

}  // namespace

int main() {
  std::ifstream input(WBC_MODEL_GOLDEN_PATH);
  std::string header;
  int case_count = 0;
  input >> header >> case_count;
  assert(input && header == "WBC_MODEL_GOLDEN_V1" && case_count == 8);
  wheel_leg::NominalWbcModel model;
  for (int index = 0; index < case_count; ++index) {
    std::string case_id;
    wheel_leg::RobotState state;
    input >> case_id;
    for (double &value : state.base_position_n_m) input >> value;
    for (double &value : state.q_n_from_b) input >> value;
    for (double &value : state.base_linear_velocity_n_m_s) input >> value;
    for (double &value : state.base_angular_velocity_n_rad_s) input >> value;
    for (double &value : state.joint_position_rad) input >> value;
    for (double &value : state.joint_velocity_rad_s) input >> value;
    state.contact_state = {wheel_leg::ContactState::kContact,
                           wheel_leg::ContactState::kContact};
    const auto result = model.evaluate(state);
    if (case_id == "dynamic_tick_271") {
      assert(result.status ==
             wheel_leg::NominalWbcModel::Status::kOutsideWorkspace);
      double ignored = 0.0;
      for (int value = 0; value < 652; ++value) input >> ignored;
      continue;
    }
    if (!result.ok()) {
      std::cerr << case_id << " status=" << static_cast<int>(result.status)
                << " closure=" << result.diagnostics.closure_residual_m
                << " condition=" << result.diagnostics.passive_condition_number
                << std::endl;
    }
    assert(result.ok());
    if (case_id == "workspace_equilibrium") {
      // Independent Phase-23 MuJoCo state-oracle-v2 wheel-body positions.
      assert(std::abs(result.wheel_position_b_x_m[0] -
                      (-0.009573649495650122)) <= 2.0e-10);
      assert(std::abs(result.wheel_position_b_x_m[1] -
                      (-0.012740695843911437)) <= 2.0e-10);
      assert(std::abs(result.wheel_velocity_b_x_m_s[0]) <= 1.0e-12);
      assert(std::abs(result.wheel_velocity_b_x_m_s[1]) <= 1.0e-12);
      const double common = 0.5 *
          (result.wheel_position_b_x_m[0] +
           result.wheel_position_b_x_m[1]);
      const double differential = 0.5 *
          (result.wheel_position_b_x_m[1] -
           result.wheel_position_b_x_m[0]);
      assert(std::abs(common - (-0.01115717266978078)) <= 2.0e-10);
      assert(std::abs(differential - (-0.0015835231741306575)) <=
             2.0e-10);
      for (int side = 0; side < 2; ++side) {
        const auto rotation = result.interaction_contact_map[side]
                                  .topLeftCorner<3, 3>();
        assert((rotation.transpose() * rotation - Eigen::Matrix3d::Identity())
                   .cwiseAbs().maxCoeff() <= 1.0e-12);
        assert(std::abs(rotation.determinant() - 1.0) <= 1.0e-12);
        // A stationary wheel with no ground wrench pulls downward on its
        // parent, matching the historical follower-on-base sign.
        assert(result.interaction_bias[side][2] < 0.0);
      }
      assert(std::abs(result.interaction_bias[0][2] -
                      result.interaction_bias[1][2]) <= 1.0e-10);
    }
    double reconstruction_error = maximumError(
        result.native_joint_position_rad, input);
    double reduction_error = maximumError(result.reduction, input);
    double mass_error = maximumError(result.mass, input);
    double bias_error = maximumError(result.bias, input);
    double actuation_error = maximumError(result.actuation, input);
    double wrench_error = 0.0;
    double contact_error = 0.0;
    double contact_bias_error = 0.0;
    for (int side = 0; side < 2; ++side) {
      wrench_error = std::max(
          wrench_error, maximumError(result.wrench_map[side], input));
    }
    for (int side = 0; side < 2; ++side) {
      contact_error = std::max(
          contact_error, maximumError(result.contact_jacobian[side], input));
    }
    for (int side = 0; side < 2; ++side) {
      contact_bias_error = std::max(
          contact_bias_error, maximumError(result.contact_bias[side], input));
    }
    std::cout << case_id << " reconstruction=" << reconstruction_error
              << " reduction=" << reduction_error << " mass=" << mass_error
              << " bias=" << bias_error << " actuation=" << actuation_error
              << " wrench=" << wrench_error << " contact=" << contact_error
              << " contact_bias=" << contact_bias_error << std::endl;
    assert(reconstruction_error <= 1.0e-9);
    assert(reduction_error <= 1.0e-8);
    assert(mass_error <= 1.0e-8);
    assert(bias_error <= 2.0e-5);
    assert(actuation_error <= 1.0e-10);
    assert(wrench_error <= 1.0e-8);
    assert(contact_error <= 1.0e-8);
    assert(contact_bias_error <= 2.0e-5);
  }
  wheel_leg::RobotState invalid;
  invalid.joint_position_rad[0] = 10.0;
  assert(model.evaluate(invalid).status ==
         wheel_leg::NominalWbcModel::Status::kOutsideWorkspace);
  invalid.joint_position_rad[0] = std::numeric_limits<double>::quiet_NaN();
  assert(model.evaluate(invalid).status ==
         wheel_leg::NominalWbcModel::Status::kInvalidState);
  return 0;
}
