#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cmath>
#include <limits>

#include "wheel_leg_core/dense_qp_solver.hpp"

namespace {

using wheel_leg::DenseQpSolver;

Eigen::MatrixXd identityH() {
  return Eigen::MatrixXd::Identity(DenseQpSolver::kVariableCount,
                                   DenseQpSolver::kVariableCount);
}

Eigen::VectorXd zeroG() {
  return Eigen::VectorXd::Zero(DenseQpSolver::kVariableCount);
}

void expectNear(double actual, double expected, double tolerance = 2.0e-5) {
  assert(std::abs(actual - expected) <= tolerance);
}

DenseQpSolver setupOneRow(const Eigen::VectorXd& g, double coefficient,
                          double lower, double upper,
                          DenseQpSolver::Settings settings = {}) {
  DenseQpSolver solver(settings);
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(1, DenseQpSolver::kVariableCount);
  a(0, 0) = coefficient;
  Eigen::VectorXd l(1);
  Eigen::VectorXd u(1);
  l[0] = lower;
  u[0] = upper;
  assert(solver.setup(identityH(), g, a, l, u) ==
         DenseQpSolver::Status::kConverged);
  return solver;
}

}  // namespace

int main() {
  using Status = DenseQpSolver::Status;
  using StartMode = DenseQpSolver::StartMode;

  // Unconstrained golden solution: x = -g for H = I.
  Eigen::VectorXd g = zeroG();
  g[0] = -2.0;
  g[5] = 0.5;
  g[DenseQpSolver::kVariableCount - 1] = -0.75;
  DenseQpSolver unconstrained;
  Eigen::MatrixXd empty_a(0, DenseQpSolver::kVariableCount);
  Eigen::VectorXd empty(0);
  assert(unconstrained.setup(identityH(), g, empty_a, empty, empty) ==
         Status::kConverged);
  const auto unconstrained_result = unconstrained.solve(StartMode::kCold);
  assert(unconstrained_result.converged());
  expectNear(unconstrained_result.x[0], 2.0, 1.0e-9);
  expectNear(unconstrained_result.x[5], -0.5, 1.0e-9);
  expectNear(unconstrained_result.x[DenseQpSolver::kVariableCount - 1], 0.75,
             1.0e-9);
  assert(unconstrained_result.stationarity_residual < 1.0e-9);

  // Non-diagonal SPD golden case; the 2x2 analytic solution is
  // x = [5/11, -9/11] for H=[[4,1],[1,3]], g=[-1,2].
  Eigen::MatrixXd non_diagonal_h = identityH();
  non_diagonal_h(0, 0) = 4.0;
  non_diagonal_h(0, 1) = 1.0;
  non_diagonal_h(1, 0) = 1.0;
  non_diagonal_h(1, 1) = 3.0;
  Eigen::VectorXd non_diagonal_g = zeroG();
  non_diagonal_g[0] = -1.0;
  non_diagonal_g[1] = 2.0;
  DenseQpSolver non_diagonal;
  assert(non_diagonal.setup(non_diagonal_h, non_diagonal_g, empty_a, empty,
                            empty) == Status::kConverged);
  const auto non_diagonal_result = non_diagonal.solve(StartMode::kCold);
  assert(non_diagonal_result.converged());
  expectNear(non_diagonal_result.x[0], 5.0 / 11.0, 1.0e-9);
  expectNear(non_diagonal_result.x[1], -9.0 / 11.0, 1.0e-9);
  assert(non_diagonal_result.stationarity_residual < 1.0e-9);

  // Equality, lower, and upper active bounds.
  g = zeroG();
  g[0] = -3.0;
  auto equality = setupOneRow(g, 1.0, 1.25, 1.25);
  expectNear(equality.solve(StartMode::kCold).x[0], 1.25);

  auto lower_active = setupOneRow(zeroG(), 1.0, 1.0, 3.0);
  expectNear(lower_active.solve(StartMode::kCold).x[0], 1.0);
  g = zeroG();
  g[0] = -3.0;
  auto upper_active = setupOneRow(g, 1.0, -2.0, 1.0);
  expectNear(upper_active.solve(StartMode::kCold).x[0], 1.0);

  // Mixed equality and inequalities.
  Eigen::MatrixXd a = Eigen::MatrixXd::Zero(3, DenseQpSolver::kVariableCount);
  a(0, 0) = 1.0;
  a(1, 1) = 1.0;
  a(2, 0) = 1.0;
  a(2, 1) = 1.0;
  Eigen::VectorXd l(3);
  Eigen::VectorXd u(3);
  l << 0.25, -1.0, 1.0;
  u << 0.25, 2.0, 1.0;
  DenseQpSolver mixed;
  assert(mixed.setup(identityH(), zeroG(), a, l, u) == Status::kConverged);
  const auto mixed_result = mixed.solve(StartMode::kCold);
  assert(mixed_result.converged());
  expectNear(mixed_result.x[0], 0.25);
  expectNear(mixed_result.x[1], 0.75);

  // Input validation rejects inconsistent, non-finite, and non-convex data.
  DenseQpSolver invalid;
  l[0] = 2.0;
  u[0] = 1.0;
  assert(invalid.setup(identityH(), zeroG(), a, l, u) == Status::kInvalidInput);
  l[0] = 0.25;
  u[0] = 0.25;
  Eigen::VectorXd nonfinite_g = zeroG();
  nonfinite_g[0] = std::numeric_limits<double>::quiet_NaN();
  assert(invalid.setup(identityH(), nonfinite_g, a, l, u) == Status::kInvalidInput);
  Eigen::MatrixXd indefinite = identityH();
  indefinite(0, 0) = -1.0;
  assert(invalid.setup(indefinite, zeroG(), a, l, u) == Status::kInvalidInput);
  const auto rejected = invalid.solve(StartMode::kCold);
  assert(rejected.status == Status::kInvalidInput);
  assert(rejected.x.isZero(0.0));

  // Repeated cold starts are deterministic and warm convergence preserves result.
  g = zeroG();
  g[0] = -3.0;
  auto deterministic = setupOneRow(g, 1.0, -2.0, 1.0);
  const auto first_cold = deterministic.solve(StartMode::kCold);
  const auto warm = deterministic.solve(StartMode::kWarm);
  const auto second_cold = deterministic.solve(StartMode::kCold);
  assert(first_cold.converged() && warm.converged() && second_cold.converged());
  assert(first_cold.x.isApprox(second_cold.x, 1.0e-12));
  assert(first_cold.x.isApprox(warm.x, 2.0e-5));

  // setup(kWarm) retains compatible state across a changed QP, then projects
  // the retained auxiliary state onto the new bounds.
  Eigen::MatrixXd updated_a =
      Eigen::MatrixXd::Zero(1, DenseQpSolver::kVariableCount);
  updated_a(0, 0) = 2.0;
  Eigen::VectorXd updated_lower(1);
  Eigen::VectorXd updated_upper(1);
  updated_lower[0] = -4.0;
  updated_upper[0] = 1.0;
  Eigen::VectorXd updated_g = zeroG();
  updated_g[0] = -4.0;
  assert(deterministic.setup(identityH(), updated_g, updated_a, updated_lower,
                             updated_upper,
                             DenseQpSolver::SetupMode::kWarm) ==
         Status::kConverged);
  const auto updated_warm = deterministic.solve(StartMode::kWarm);
  assert(updated_warm.converged());
  expectNear(updated_warm.x[0], 0.5, 5.0e-4);

  // A different row count cannot retain a stale warm state.
  Eigen::MatrixXd two_rows =
      Eigen::MatrixXd::Zero(2, DenseQpSolver::kVariableCount);
  two_rows(0, 0) = 1.0;
  two_rows(1, 1) = 1.0;
  Eigen::VectorXd two_lower(2);
  Eigen::VectorXd two_upper(2);
  two_lower << -1.0, -1.0;
  two_upper << 1.0, 1.0;
  assert(deterministic.setup(identityH(), zeroG(), two_rows, two_lower,
                             two_upper, DenseQpSolver::SetupMode::kWarm) ==
         Status::kConverged);
  const auto changed_rows = deterministic.solve(StartMode::kWarm);
  assert(changed_rows.converged());
  assert(changed_rows.x.isZero(2.0e-5));

  // Moving an equality row to another index is incompatible with a retained
  // dual candidate, so setup(kWarm) must rebuild and still solve correctly.
  Eigen::MatrixXd changed_mask = two_rows;
  Eigen::VectorXd changed_mask_lower(2);
  Eigen::VectorXd changed_mask_upper(2);
  changed_mask_lower << -1.0, 0.0;
  changed_mask_upper << 1.0, 0.0;
  assert(deterministic.setup(identityH(), zeroG(), changed_mask,
                             changed_mask_lower, changed_mask_upper,
                             DenseQpSolver::SetupMode::kWarm) ==
         Status::kConverged);
  const auto changed_mask_result = deterministic.solve(StartMode::kWarm);
  assert(changed_mask_result.converged());
  assert(changed_mask_result.x.isZero(2.0e-5));

  // reset removes the previous candidate; a warm request after reset is cold.
  deterministic.reset();
  assert(deterministic.setup(identityH(), zeroG(), changed_mask,
                             changed_mask_lower, changed_mask_upper,
                             DenseQpSolver::SetupMode::kWarm) ==
         Status::kConverged);
  const auto reset_result = deterministic.solve(StartMode::kWarm);
  assert(reset_result.converged());
  assert(reset_result.x.isZero(2.0e-5));

  DenseQpSolver::Settings one_iteration;
  one_iteration.maximum_iterations = 1;
  auto limited = setupOneRow(g, 1.0, -2.0, 1.0, one_iteration);
  const auto maximum_iterations = limited.solve(StartMode::kCold);
  assert(maximum_iterations.status == Status::kMaximumIterations);
  assert(maximum_iterations.x.isZero(0.0));

  // Contradictory equalities exercise ProxQP's primal-infeasibility mapping.
  Eigen::MatrixXd contradictory =
      Eigen::MatrixXd::Zero(2, DenseQpSolver::kVariableCount);
  contradictory(0, 0) = 1.0;
  contradictory(1, 0) = 1.0;
  Eigen::VectorXd contradictory_target(2);
  contradictory_target << 0.0, 1.0;
  DenseQpSolver infeasible;
  assert(infeasible.setup(identityH(), zeroG(), contradictory,
                          contradictory_target, contradictory_target) ==
         Status::kConverged);
  const auto primal_infeasible = infeasible.solve(StartMode::kCold);
  assert(primal_infeasible.status == Status::kPrimalInfeasible);
  assert(primal_infeasible.x.isZero(0.0));

  // A linear objective with no curvature or bounds is unbounded and exercises
  // ProxQP's dual-infeasibility mapping.
  Eigen::MatrixXd zero_h = Eigen::MatrixXd::Zero(
      DenseQpSolver::kVariableCount, DenseQpSolver::kVariableCount);
  Eigen::VectorXd unbounded_g = zeroG();
  unbounded_g[0] = -1.0;
  DenseQpSolver unbounded;
  assert(unbounded.setup(zero_h, unbounded_g, empty_a, empty, empty) ==
         Status::kConverged);
  const auto dual_infeasible = unbounded.solve(StartMode::kCold);
  assert(dual_infeasible.status == Status::kDualInfeasible);
  assert(dual_infeasible.x.isZero(0.0));
  return 0;
}
