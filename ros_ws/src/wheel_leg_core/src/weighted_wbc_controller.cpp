#include "wheel_leg_core/weighted_wbc_controller.hpp"

#include <algorithm>
#include <cmath>

#include "nominal_wbc_profile_data.hpp"

namespace wheel_leg {

WeightedWbcController::WeightedWbcController()
    : solver_([] {
        DenseQpSolver::Settings settings;
        settings.rho = 0.15;
        settings.sigma = 1.0e-6;
        settings.absolute_tolerance = 1.0e-8;
        settings.relative_tolerance = 1.0e-8;
        settings.maximum_iterations = 10000;
        return settings;
      }()) {}

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
  const auto problem = problem_.assemble(model, reference);
  if (!problem.ok()) {
    output.status = problem.status == WeightedWbcProblem::Status::kNonFinite
                        ? Status::kNonFinite
                        : Status::kProblemRejected;
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
  for (int joint = 0; joint < 6; ++joint) {
    output.torque_nm[static_cast<std::size_t>(joint)] =
        phase21_profile::kVariableScale[12 + joint] * solved.x[12 + joint];
  }
  for (int index = 0; index < 42; ++index) {
    output.physical_solution[index] =
        phase21_profile::kVariableScale[index] * solved.x[index];
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
  record_task(Task::kContact, contact / 10.0);
  record_task(Task::kBaseX, (Eigen::Matrix<double, 1, 1>() <<
      (output.physical_solution[0] - reference.base_x_acceleration_m_s2) / 10.0).finished());
  record_task(Task::kHeight, (Eigen::Matrix<double, 1, 1>() <<
      (output.physical_solution[2] - reference.base_height_acceleration_m_s2) / 10.0).finished());
  record_task(Task::kOrientation,
      (output.physical_solution.segment<3>(3) - reference.orientation_acceleration_rad_s2) / 20.0);
  Eigen::Matrix<double, 4, 1> leg;
  for (int index = 0; index < 4; ++index) leg[index] = output.physical_solution[6 + (index < 2 ? index : index + 1)];
  record_task(Task::kLeg, (leg - reference.leg_acceleration_rad_s2) / 50.0);
  Eigen::Matrix<double, 12, 1> wrench;
  for (int side = 0; side < 2; ++side) {
    wrench.segment<6>(6 * side) = model.wrench_flu_map[side] *
        output.physical_solution.segment<6>(18 + 6 * side) -
        output.physical_solution.segment<6>(30 + 6 * side) -
        reference.interaction_wrench_flu.segment<6>(6 * side);
  }
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
