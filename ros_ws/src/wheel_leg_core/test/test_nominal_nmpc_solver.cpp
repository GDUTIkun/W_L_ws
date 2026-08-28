#include "wheel_leg_core/nominal_nmpc_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>

namespace {

constexpr double kHeightM = 0.31543998403249462;

wheel_leg::NominalNmpcProblem equilibriumProblem() {
  wheel_leg::NominalNmpcProblem problem;
  problem.state[2] = kHeightM;
  problem.reference = problem.state;
  return problem;
}

int require(bool condition, const char *message) {
  if (condition) return 0;
  std::cerr << message << '\n';
  return 1;
}

}  // namespace

int main() {
  wheel_leg::NominalNmpcSolver solver;
  if (require(solver.ready(), "solver did not initialize")) return 1;

  const auto equilibrium = solver.solve(equilibriumProblem());
  if (!equilibrium.ok()) {
    std::cerr << "equilibrium status=" << static_cast<int>(equilibrium.status)
              << " acados=" << equilibrium.acados_status
              << " residuals=" << equilibrium.stationarity_residual << ','
              << equilibrium.dynamics_residual << ','
              << equilibrium.inequality_residual << ','
              << equilibrium.complementarity_residual
              << " defect=" << equilibrium.first_step_defect << '\n';
  }
  if (require(equilibrium.ok(), "equilibrium solve failed") ||
      require(equilibrium.wrench_flu.allFinite(), "non-finite wrench") ||
      require(equilibrium.first_step_defect <= 1.0e-4,
              "equilibrium dynamics defect")) {
    return 1;
  }

  auto perturbed = equilibriumProblem();
  perturbed.state[0] = 0.005;
  const auto first = solver.solve(perturbed);
  if (!first.ok()) {
    std::cerr << "perturbed status=" << static_cast<int>(first.status)
              << " acados=" << first.acados_status
              << " residuals=" << first.stationarity_residual << ','
              << first.dynamics_residual << ',' << first.inequality_residual
              << ',' << first.complementarity_residual
              << " defect=" << first.first_step_defect << '\n';
  }
  if (require(first.ok(), "perturbed solve failed")) return 1;
  solver.reset();
  const auto reset_first = solver.solve(perturbed);
  solver.reset();
  const auto reset_second = solver.solve(perturbed);
  if (require(reset_first.ok() && reset_second.ok(), "reset solve failed") ||
      require((reset_first.wrench_flu - reset_second.wrench_flu)
                      .cwiseAbs()
                      .maxCoeff() <= 1.0e-12,
              "cold reset is not deterministic")) {
    return 1;
  }

  auto invalid = equilibriumProblem();
  invalid.state[0] = std::numeric_limits<double>::quiet_NaN();
  if (require(
          solver.solve(invalid).status ==
              wheel_leg::NominalNmpcSolver::Status::kInvalidInput,
          "non-finite input was accepted")) {
    return 1;
  }

  double maximum_ms = 0.0;
  for (int iteration = 0; iteration < 1000; ++iteration) {
    auto dynamic = equilibriumProblem();
    dynamic.state[0] = 0.004 * std::sin(0.01 * iteration);
    dynamic.state[6] = 0.02 * std::cos(0.01 * iteration);
    const auto start = std::chrono::steady_clock::now();
    const auto result = solver.solve(dynamic);
    const auto end = std::chrono::steady_clock::now();
    if (require(result.ok(), "dynamic warm solve failed")) return 1;
    maximum_ms = std::max(
        maximum_ms,
        std::chrono::duration<double, std::milli>(end - start).count());
  }
  std::cout << "phase23 nominal NMPC solver: PASS, max_ms=" << maximum_ms
            << '\n';
  return 0;
}
