#include "wheel_leg_core/weighted_wbc_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "nominal_wbc_profile_data.hpp"

namespace wheel_leg {

namespace {

bool usesMinimalInteractionWrench(WeightedWbcProfile profile) {
  return profile == WeightedWbcProfile::kPhase27Minimal ||
         profile == WeightedWbcProfile::kPhase33ZetaManifold ||
         profile == WeightedWbcProfile::kPhase34XiTracking ||
         profile == WeightedWbcProfile::kPhase43NativeWheelRate ||
         profile == WeightedWbcProfile::kPhase43XiAndNativeWheelRate ||
         profile == WeightedWbcProfile::kPhase45ContactConsistentRolling ||
         profile == WeightedWbcProfile::kPhase46HipCommonSafeRolling ||
         profile == WeightedWbcProfile::kPhase46HipCommonIncrementLimitedRolling ||
         profile == WeightedWbcProfile::kPhase46PointRealizableRolling;
}

}  // namespace

WeightedWbcController::WeightedWbcController(WeightedWbcProfile profile)
    : solver_([] {
        DenseQpSolver::Settings settings;
        settings.absolute_tolerance = 1.0e-8;
        settings.relative_tolerance = 1.0e-8;
        settings.maximum_iterations = 10000;
        return settings;
      }()),
      profile_(profile) {}

void WeightedWbcController::reset() {
  solver_.reset();
  warm_start_available_ = false;
}

WeightedWbcController::Result WeightedWbcController::step(
    const RobotState &state, const WbcReference &reference) {
  Result output;
  const auto model = model_.evaluate(state);
  output.model_status = model.status;
  output.model_diagnostics = model.diagnostics;
  if (!model.ok()) {
    output.status = Status::kModelRejected;
    reset();
    return output;
  }
  output.reduction = model.reduction;
  output.reduction_bias = model.reduction_bias;
  output.contact_wrench_map = model.wrench_map;
  output.wheel_longitudinal_acceleration_map =
      model.wheel_longitudinal_acceleration_map;
  for (int side = 0; side < 2; ++side) {
    output.wheel_longitudinal_acceleration_bias_m_s2[side] =
        model.wheel_longitudinal_acceleration_bias_m_s2[side];
  }
  const auto problem = problem_.assemble(model, reference, profile_);
  if (!problem.ok()) {
    output.status = problem.status == WeightedWbcProblem::Status::kNonFinite
                        ? Status::kNonFinite
                        : Status::kProblemRejected;
    reset();
    return output;
  }
  int equality_count = 0;
  for (Eigen::Index row = 0; row < problem.a.rows(); ++row) {
    equality_count += problem.lower[row] == problem.upper[row] ? 1 : 0;
  }
  const int expected_equalities =
      profile_ == WeightedWbcProfile::kPhase46HipCommonIncrementLimitedRolling &&
              reference.hip_common_increment_limit_active
          ? 13 : 12;
  if (problem.a.rows() != WeightedWbcProblem::kConstraintCount ||
      equality_count != expected_equalities) {
    output.solver_status = DenseQpSolver::Status::kInvalidInput;
    output.status = Status::kSolverRejected;
    reset();
    return output;
  }
  const auto setup_mode = warm_start_available_
                              ? DenseQpSolver::SetupMode::kWarm
                              : DenseQpSolver::SetupMode::kCold;
  output.solver_status = solver_.setup(
      problem.h, problem.g, problem.a, problem.lower, problem.upper,
      setup_mode);
  if (output.solver_status != DenseQpSolver::Status::kConverged) {
    output.status = Status::kSolverRejected;
    reset();
    return output;
  }
  const auto solved = solver_.solve(
      warm_start_available_ ? DenseQpSolver::StartMode::kWarm
                            : DenseQpSolver::StartMode::kCold);
  output.solver_status = solved.status;
  output.iterations = solved.iterations;
  output.stationarity_residual = solved.stationarity_residual;
  output.primal_residual = solved.primal_residual;
  output.dual_residual = solved.dual_residual;
  if (!solved.converged() || !solved.x.allFinite()) {
    output.status = solved.x.allFinite() ? Status::kSolverRejected
                                         : Status::kNonFinite;
    reset();
    return output;
  }
  const auto ax = problem.a * solved.x;
  output.hard_violation = std::max(
      0.0, std::max((problem.lower - ax).maxCoeff(),
                    (ax - problem.upper).maxCoeff()));
  if (!std::isfinite(output.hard_violation) ||
      output.hard_violation > 2.0e-7) {
    output.status = std::isfinite(output.hard_violation)
                        ? Status::kHardViolation
                        : Status::kNonFinite;
    reset();
    return output;
  }
  constexpr std::array<std::pair<int, int>, 3> inequality_ranges{{
      {12, 18}, {18, 92}, {92, 104}}};
  for (std::size_t group = 0; group < inequality_ranges.size(); ++group) {
    output.minimum_inequality_margin[group] =
        std::numeric_limits<double>::infinity();
    for (int row = inequality_ranges[group].first;
         row < inequality_ranges[group].second; ++row) {
      const double lower_margin = ax[row] - problem.lower[row];
      const double upper_margin = problem.upper[row] - ax[row];
      const double margin = std::min(lower_margin, upper_margin);
      output.minimum_inequality_margin[group] =
          std::min(output.minimum_inequality_margin[group], margin);
      output.active_inequality_count[group] += margin <= 1.0e-7 ? 1 : 0;
      output.inequality_active_side[row] =
          (lower_margin <= 1.0e-7 ? 1 : 0) |
          (upper_margin <= 1.0e-7 ? 2 : 0);
    }
  }
  for (int joint = 0; joint < 6; ++joint) {
    output.torque_nm[static_cast<std::size_t>(joint)] =
        phase21_profile::kVariableScale[12 + joint] * solved.x[12 + joint];
  }
  for (int index = 0; index < 42; ++index) {
    output.physical_solution[index] =
        phase21_profile::kVariableScale[index] * solved.x[index];
  }
  if (profile_ == WeightedWbcProfile::kPhase46PointRealizableRolling ||
      profile_ == WeightedWbcProfile::kPhase46ConstraintConsistentLegClosureReaction) {
    for (int side = 0; side < 2; ++side) {
      output.physical_solution.segment<6>(18 + 6 * side) =
          model.point_force_wrench_projector[side] *
          output.physical_solution.segment<6>(18 + 6 * side);
    }
  }
  const auto record_task = [&output](Task task, const auto &residual) {
    const auto index = static_cast<std::size_t>(task);
    output.task_max_abs_normalized_residual[index] =
        residual.cwiseAbs().maxCoeff();
    output.task_normalized_squared_cost[index] = residual.squaredNorm();
  };
  Eigen::Matrix<double, 6, 1> contact;
  contact << model.contact_jacobian[0] * output.physical_solution.head<12>() +
                 model.contact_bias[0],
      model.contact_jacobian[1] * output.physical_solution.head<12>() +
          model.contact_bias[1];
  output.contact_task_residual = contact;
  record_task(Task::kContact, contact / 10.0);
  for (int side = 0; side < 2; ++side) {
    output.wheel_position_b_x_m[side] = model.wheel_position_b_x_m[side];
    output.wheel_velocity_b_x_m_s[side] =
        model.wheel_velocity_b_x_m_s[side];
    output.wheel_longitudinal_acceleration_m_s2[side] =
        (model.wheel_longitudinal_acceleration_map[side] *
         output.physical_solution.head<12>())(0) +
        model.wheel_longitudinal_acceleration_bias_m_s2[side];
    output.wheel_position_b_z_m[side] = model.wheel_position_b_z_m[side];
    output.wheel_velocity_b_z_m_s[side] =
        model.wheel_velocity_b_z_m_s[side];
    output.wheel_vertical_acceleration_m_s2[side] =
        (model.wheel_vertical_acceleration_map[side] *
         output.physical_solution.head<12>())(0) +
        model.wheel_vertical_acceleration_bias_m_s2[side];
  }
  if (profile_ == WeightedWbcProfile::kPhase33ZetaManifold) {
    record_task(Task::kWheelVerticalManifold,
                output.wheel_vertical_acceleration_m_s2 -
                    reference.wheel_vertical_acceleration_m_s2);
  }
  if (profile_ == WeightedWbcProfile::kPhase34XiTracking ||
      profile_ == WeightedWbcProfile::kPhase43XiAndNativeWheelRate ||
      profile_ == WeightedWbcProfile::kPhase45ContactConsistentRolling ||
      profile_ == WeightedWbcProfile::kPhase46HipCommonSafeRolling ||
      profile_ == WeightedWbcProfile::kPhase46HipCommonIncrementLimitedRolling ||
      profile_ == WeightedWbcProfile::kPhase46PointRealizableRolling ||
      profile_ == WeightedWbcProfile::kPhase46ConstraintConsistentLegClosureReaction) {
    record_task(Task::kWheelLongitudinalTracking,
                output.wheel_longitudinal_acceleration_m_s2 -
                    reference.wheel_longitudinal_acceleration_m_s2);
  }
  if (profile_ == WeightedWbcProfile::kPhase45ContactConsistentRolling ||
      profile_ == WeightedWbcProfile::kPhase46HipCommonSafeRolling ||
      profile_ == WeightedWbcProfile::kPhase46HipCommonIncrementLimitedRolling ||
      profile_ == WeightedWbcProfile::kPhase46PointRealizableRolling ||
      profile_ == WeightedWbcProfile::kPhase46ConstraintConsistentLegClosureReaction) {
    output.rolling_velocity_m_s = reference.rolling_velocity_m_s;
    output.rolling_task_active = reference.rolling_task_active;
    Eigen::Vector2d residual = Eigen::Vector2d::Zero();
    for (int side = 0; side < 2; ++side) {
      const auto rolling_map =
          profile_ == WeightedWbcProfile::kPhase46HipCommonSafeRolling
              ? hipCommonSafeRollingMap(reference.rolling_acceleration_map[side])
              : reference.rolling_acceleration_map[side];
      output.rolling_acceleration_m_s2[side] =
          (rolling_map *
           output.physical_solution.head<12>())(0) +
          reference.rolling_acceleration_bias_m_s2[side];
      if (reference.rolling_task_active[side]) {
        residual[side] = output.rolling_acceleration_m_s2[side] -
                         reference.rolling_acceleration_m_s2[side];
      }
    }
    record_task(Task::kContactConsistentRolling, residual);
  }
  if (profile_ == WeightedWbcProfile::kPhase43NativeWheelRate ||
      profile_ == WeightedWbcProfile::kPhase43XiAndNativeWheelRate) {
    Eigen::Vector2d wheel_acceleration;
    wheel_acceleration << output.physical_solution[8],
        output.physical_solution[11];
    record_task(Task::kNativeWheelRate,
                (wheel_acceleration -
                 reference.wheel_joint_acceleration_rad_s2) / 20.0);
  }
  if (profile_ == WeightedWbcProfile::kNominal) {
    record_task(Task::kBaseX, (Eigen::Matrix<double, 1, 1>() <<
        (output.physical_solution[0] - reference.base_x_acceleration_m_s2) /
            10.0).finished());
    record_task(Task::kHeight, (Eigen::Matrix<double, 1, 1>() <<
        (output.physical_solution[2] -
         reference.base_height_acceleration_m_s2) / 10.0).finished());
    record_task(Task::kOrientation,
        (output.physical_solution.segment<3>(3) -
         reference.orientation_acceleration_rad_s2) / 20.0);
    Eigen::Matrix<double, 4, 1> leg;
    for (int index = 0; index < 4; ++index) {
      leg[index] = output.physical_solution[6 + (index < 2 ? index : index + 1)];
    }
    record_task(Task::kLeg, (leg - reference.leg_acceleration_rad_s2) / 50.0);
  }
  Eigen::Matrix<double, 12, 1> wrench;
  for (int side = 0; side < 2; ++side) {
    if (usesMinimalInteractionWrench(profile_)) {
      output.realized_interaction_wrench_flu.segment<6>(6 * side) =
          model.interaction_acceleration_map[side] *
              output.physical_solution.head<12>() +
          model.interaction_contact_map[side] *
              output.physical_solution.segment<6>(18 + 6 * side) +
          model.interaction_bias[side];
    } else {
      output.realized_interaction_wrench_flu.segment<6>(6 * side) =
          model.wrench_flu_map[side] *
          output.physical_solution.segment<6>(18 + 6 * side);
    }
  }
  output.signed_interaction_slack_flu = output.physical_solution.tail<12>();
  output.interaction_wrench_residual_flu =
      output.realized_interaction_wrench_flu -
      reference.interaction_wrench_flu -
      output.signed_interaction_slack_flu;
  wrench = output.interaction_wrench_residual_flu;
  Eigen::Matrix<double, 12, 1> wrench_scale;
  for (int index = 0; index < 12; ++index) wrench_scale[index] = phase21_profile::kVariableScale[30 + index % 6];
  record_task(Task::kWrenchFidelity, wrench.cwiseQuotient(wrench_scale));
  const auto slack = output.physical_solution.tail<12>().cwiseQuotient(wrench_scale);
  record_task(Task::kSlackPenalty, slack);
  output.maximum_normalized_slack = slack.cwiseAbs().maxCoeff();
  if (validateTorqueCommand(TorqueCommand{state.sample_time_ns,
                                          output.torque_nm}) !=
      ValidationError::kNone) {
    output = Result{};
    output.status = Status::kNonFinite;
    reset();
    return output;
  }
  output.status = Status::kOk;
  warm_start_available_ = true;
  return output;
}

}  // namespace wheel_leg
