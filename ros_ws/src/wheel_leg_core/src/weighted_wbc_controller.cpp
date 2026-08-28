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
