#include "wheel_leg_core/nominal_nmpc_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

namespace {

constexpr double kHeightM = 0.31543998403249462;

wheel_leg::NominalNmpcProblem equilibriumProblem() {
  wheel_leg::NominalNmpcProblem problem;
  problem.state[2] = kHeightM;
  problem.reference = problem.state;
  problem.state_envelope_center = problem.state;
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
      require(equilibrium.maximum_dynamics_defect <= 1.0e-3,
              "equilibrium dynamics defect") ||
      require(std::isfinite(equilibrium.objective), "non-finite objective") ||
      require(std::isfinite(equilibrium.projected_stationarity_residual),
              "non-finite independent stationarity")) {
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
  if (require(first.state_bound_violation <= 1.0e-8,
              "predicted state envelope violation")) {
    return 1;
  }
  auto outside_envelope = equilibriumProblem();
  outside_envelope.state_envelope_center[0] = 1.0;
  const auto rejected = solver.solve(outside_envelope);
  if (require(!rejected.ok(), "infeasible envelope was accepted")) return 1;
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

  auto benchmark = [&](const char *name, auto solve_once) {
    std::vector<double> samples;
    samples.reserve(1000);
    for (int iteration = 0; iteration < 1000; ++iteration) {
      const auto start = std::chrono::steady_clock::now();
      if (require(solve_once(iteration), name)) std::exit(1);
      const auto end = std::chrono::steady_clock::now();
      samples.push_back(
          std::chrono::duration<double, std::milli>(end - start).count());
    }
    std::sort(samples.begin(), samples.end());
    std::cout << name << " p99_ms=" << samples[989]
              << " max_ms=" << samples.back() << '\n';
  };
  const auto deterministic_problem = perturbed;
  wheel_leg::NominalNmpcModel::Input cold_reference;
  bool have_cold_reference = false;
  double maximum_projected_stationarity = 0.0;
  double maximum_full_horizon_defect = 0.0;
  benchmark("cold", [&](int) {
    solver.reset();
    const auto result = solver.solve(deterministic_problem);
    if (!result.ok()) return false;
    maximum_projected_stationarity = std::max(
        maximum_projected_stationarity,
        result.projected_stationarity_residual);
    maximum_full_horizon_defect = std::max(
        maximum_full_horizon_defect, result.maximum_dynamics_defect);
    if (!have_cold_reference) {
      cold_reference = result.wrench_flu;
      have_cold_reference = true;
    }
    return (result.wrench_flu - cold_reference).cwiseAbs().maxCoeff() <= 1.0e-12;
  });
  solver.reset();
  if (require(solver.solve(equilibriumProblem()).ok(), "warm preload failed")) {
    return 1;
  }
  benchmark("repeated_warm", [&](int) {
    const auto result = solver.solve(equilibriumProblem());
    maximum_projected_stationarity = std::max(
        maximum_projected_stationarity,
        result.projected_stationarity_residual);
    maximum_full_horizon_defect = std::max(
        maximum_full_horizon_defect, result.maximum_dynamics_defect);
    return result.ok();
  });
  double maximum_ms = 0.0;
  for (int iteration = 0; iteration < 1000; ++iteration) {
    auto dynamic = equilibriumProblem();
    dynamic.state[0] = 0.004 * std::sin(0.01 * iteration);
    dynamic.state[6] = 0.02 * std::cos(0.01 * iteration);
    const auto start = std::chrono::steady_clock::now();
    const auto result = solver.solve(dynamic);
    const auto end = std::chrono::steady_clock::now();
    if (require(result.ok(), "dynamic warm solve failed")) return 1;
    maximum_projected_stationarity = std::max(
        maximum_projected_stationarity,
        result.projected_stationarity_residual);
    maximum_full_horizon_defect = std::max(
        maximum_full_horizon_defect, result.maximum_dynamics_defect);
    maximum_ms = std::max(
        maximum_ms,
        std::chrono::duration<double, std::milli>(end - start).count());
  }
  if (require(maximum_projected_stationarity <= 0.05,
              "independent projected stationarity gate") ||
      require(maximum_full_horizon_defect <= 1.0e-3,
              "independent full-horizon dynamics gate")) {
    return 1;
  }
  std::cout << "dynamic_warm max_ms=" << maximum_ms
            << " equilibrium_projected_stationarity="
            << equilibrium.projected_stationarity_residual
            << " maximum_projected_stationarity="
            << maximum_projected_stationarity
            << " maximum_full_horizon_defect="
            << maximum_full_horizon_defect
            << "\nphase23 nominal NMPC solver: PASS"
            << '\n';
  return 0;
}
