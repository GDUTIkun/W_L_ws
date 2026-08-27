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
  DenseQpSolver unconstrained;
  Eigen::MatrixXd empty_a(0, DenseQpSolver::kVariableCount);
  Eigen::VectorXd empty(0);
  assert(unconstrained.setup(identityH(), g, empty_a, empty, empty) ==
         Status::kConverged);
  const auto unconstrained_result = unconstrained.solve(StartMode::kCold);
  assert(unconstrained_result.converged());
  expectNear(unconstrained_result.x[0], 2.0, 1.0e-9);
  expectNear(unconstrained_result.x[5], -0.5, 1.0e-9);
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

  DenseQpSolver::Settings one_iteration;
  one_iteration.maximum_iterations = 1;
  auto limited = setupOneRow(g, 1.0, -2.0, 1.0, one_iteration);
  const auto maximum_iterations = limited.solve(StartMode::kCold);
  assert(maximum_iterations.status == Status::kMaximumIterations);
  assert(maximum_iterations.x.isZero(0.0));
  return 0;
}
