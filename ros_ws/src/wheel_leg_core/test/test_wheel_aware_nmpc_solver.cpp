#include "wheel_leg_core/wheel_aware_nmpc_solver.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

namespace {

wheel_leg::WheelAwareNmpcProblem equilibriumProblem() {
  wheel_leg::WheelAwareNmpcProblem problem;
  problem.state << -0.077378152, 0.00000081, 0.31543998403249462,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      -0.009573649495650122, -0.012740695843911437, 0.0, 0.0;
  problem.reference = problem.state;
  problem.state_envelope_center = problem.state;
  return problem;
}

int require(bool condition, const std::string &message) {
  if (condition) return 0;
  std::cerr << message << '\n';
  return 1;
}

void printFailure(const std::string &name,
                  const wheel_leg::WheelAwareNmpcSolver::Result &result) {
  if (result.ok()) return;
  std::cerr << name << " status=" << static_cast<int>(result.status)
            << " acados=" << result.acados_status
            << " residuals=" << result.stationarity_residual << ','
            << result.dynamics_residual << ',' << result.inequality_residual
            << ',' << result.complementarity_residual
            << " defect=" << result.maximum_dynamics_defect
            << " projected=" << result.projected_stationarity_residual
            << " bounds=" << result.input_bound_violation << ','
            << result.state_bound_violation << '\n';
}

}  // namespace

int main() {
  wheel_leg::WheelAwareNmpcSolver solver;
  if (require(solver.ready(), "solver did not initialize")) return 1;

  std::vector<std::pair<std::string, wheel_leg::WheelAwareNmpcProblem>> cases;
  cases.emplace_back("equilibrium", equilibriumProblem());
  for (double velocity : {-0.05, 0.05}) {
    auto problem = equilibriumProblem();
    problem.reference[6] = velocity;
    problem.state_envelope_center = problem.reference;
    cases.emplace_back(velocity < 0.0 ? "negative_reference" : "positive_reference", problem);
  }
  auto brake = equilibriumProblem();
  brake.state[6] = 0.05;
  cases.emplace_back("brake", brake);
  auto return_case = equilibriumProblem();
  return_case.state[0] += 0.01;
  cases.emplace_back("return", return_case);
  auto wheel_common = equilibriumProblem();
  wheel_common.state[12] += 0.005;
  wheel_common.state[13] += 0.005;
  cases.emplace_back("wheel_common", wheel_common);
  auto wheel_differential = equilibriumProblem();
  wheel_differential.state[12] -= 0.005;
  wheel_differential.state[13] += 0.005;
  cases.emplace_back("wheel_differential", wheel_differential);

  double maximum_defect = 0.0;
  double maximum_projected_stationarity = 0.0;
  for (const auto &[name, problem] : cases) {
    solver.reset();
    const auto result = solver.solve(problem);
    printFailure(name, result);
    if (require(result.ok(), name + " solve failed")) return 1;
    maximum_defect = std::max(maximum_defect, result.maximum_dynamics_defect);
    maximum_projected_stationarity = std::max(
        maximum_projected_stationarity, result.projected_stationarity_residual);
  }

  const auto deterministic_problem = cases.back().second;
  solver.reset();
  const auto first = solver.solve(deterministic_problem);
  solver.reset();
  const auto second = solver.solve(deterministic_problem);
  if (require(first.ok() && second.ok(), "reset solve failed") ||
      require((first.interaction_wrench_flu - second.interaction_wrench_flu)
                      .cwiseAbs().maxCoeff() <= 1.0e-12,
              "cold reset is not deterministic")) return 1;

  auto invalid = equilibriumProblem();
  invalid.state[12] = -0.34;
  if (require(solver.solve(invalid).status ==
                  wheel_leg::WheelAwareNmpcSolver::Status::kInvalidInput,
              "wheel workspace violation was accepted")) return 1;
  invalid = equilibriumProblem();
  invalid.state[0] = std::numeric_limits<double>::quiet_NaN();
  if (require(solver.solve(invalid).status ==
                  wheel_leg::WheelAwareNmpcSolver::Status::kInvalidInput,
              "non-finite state was accepted")) return 1;

  std::vector<double> timing_ms;
  timing_ms.reserve(300);
  solver.reset();
  for (int iteration = 0; iteration < 300; ++iteration) {
    auto problem = equilibriumProblem();
    problem.state[0] += 0.005 * std::sin(0.02 * iteration);
    problem.state[6] = 0.02 * std::cos(0.02 * iteration);
    const auto start = std::chrono::steady_clock::now();
    const auto result = solver.solve(problem);
    const auto end = std::chrono::steady_clock::now();
    printFailure("dynamic_warm", result);
    if (require(result.ok(), "dynamic warm solve failed")) return 1;
    timing_ms.push_back(std::chrono::duration<double, std::milli>(end - start).count());
    maximum_defect = std::max(maximum_defect, result.maximum_dynamics_defect);
    maximum_projected_stationarity = std::max(
        maximum_projected_stationarity, result.projected_stationarity_residual);
  }
  std::sort(timing_ms.begin(), timing_ms.end());
  if (require(timing_ms.back() <= 10.0, "10 ms combined deadline exceeded") ||
      require(maximum_defect <= 1.0e-3, "full-horizon defect gate") ||
      require(maximum_projected_stationarity <= 0.05,
              "projected stationarity gate")) return 1;
  std::cout << "phase27 wheel-aware NMPC solver: PASS p99_ms="
            << timing_ms[296] << " max_ms=" << timing_ms.back()
            << " max_defect=" << maximum_defect
            << " max_projected_stationarity="
            << maximum_projected_stationarity << '\n';
  return 0;
}
