#include "wheel_leg_core/wheel_aware_nmpc_model.hpp"
#include "wheel_leg_core/wheel_aware_nmpc_solver.hpp"

#include <Eigen/Core>

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {

using Problem = wheel_leg::WheelAwareNmpcProblem;
using Result = wheel_leg::WheelAwareNmpcSolver::Result;

struct Snapshot {
  const char *name;
  Problem problem;
};

int require(bool condition, const std::string &message) {
  if (condition) return 0;
  std::cerr << message << '\n';
  return 1;
}

Snapshot t0() {
  Snapshot result{"T0_static_tick57", {}};
  result.problem.state <<
      -0.081379914534936287, -0.00011190059041843812,
      0.31580743802191885, 0.00038873244454307675,
      -0.02951816333819236, 0.0015446074596317289,
      -0.030529760336522896, 0.0010080879707239712,
      0.0024630327685851282, -0.0033274208568253218,
      -0.16723794223552133, -0.0067247649643823321,
      -0.011606401798016974, -0.015341841720186718,
      -0.013714675018040308, -0.016120504422255863;
  result.problem.reference <<
      -0.077378152000000006, 8.0999999999999997e-07,
      0.31543998403249462, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, -0.011157172669780779,
      -0.011157172669780779, 0.0, 0.0;
  result.problem.state_envelope_center = result.problem.reference;
  return result;
}

Snapshot t1() {
  Snapshot result{"T1_straight_tick44", {}};
  result.problem.state <<
      -0.077406556449928554, -0.000340679440526264,
      0.31558716104828854, 0.0011746174923848117,
      -0.014774471915189885, 0.0023346153514229883,
      -0.002511834075725704, 0.0001010595778700446,
      0.0010320407867718082, -0.00037541994684579523,
      -0.073055830359160995, 0.002222554151408328,
      -0.012413414135494457, -0.015521716646602391,
      -0.005615101404753544, -0.024852899718801797;
  result.problem.reference <<
      -0.057578151999999987, 8.0999999999999997e-07,
      0.31543998403249462, 0.0, 0.0, 0.0,
      0.088000000000000009, 0.0, 0.0, 0.0, 0.0, 0.0,
      -0.011157172669780779, -0.011157172669780779, 0.0, 0.0;
  result.problem.state_envelope_center = result.problem.reference;
  return result;
}

Result solve(wheel_leg::WheelAwareNmpcSolver &solver, const Problem &problem) {
  solver.reset();
  return solver.solve(problem);
}

double predictedAcceleration(
    const Problem &problem, const Result &solution, int axis) {
  const auto model = wheel_leg::WheelAwareNmpcModel{}.evaluate(
      problem.state, solution.interaction_wrench_flu,
      problem.reference_rotation_n_from_b);
  if (!model.ok()) return std::numeric_limits<double>::quiet_NaN();
  return model.continuous[axis];
}

int perturbation(
    wheel_leg::WheelAwareNmpcSolver &solver, const Snapshot &snapshot,
    int state_index, double delta, int acceleration_index,
    const char *label, bool expected_negative) {
  Problem minus = snapshot.problem;
  Problem plus = snapshot.problem;
  minus.state[state_index] -= delta;
  plus.state[state_index] += delta;
  const auto negative = solve(solver, minus);
  const auto positive = solve(solver, plus);
  if (require(negative.ok() && positive.ok(),
              std::string(snapshot.name) + " " + label + " solve failed")) {
    return 1;
  }
  const double derivative =
      (predictedAcceleration(plus, positive, acceleration_index) -
       predictedAcceleration(minus, negative, acceleration_index)) /
      (2.0 * delta);
  std::cout << snapshot.name << ',' << label << ',' << derivative << '\n';
  return require(
      std::isfinite(derivative) &&
          (expected_negative ? derivative < -1.0e-3 : derivative > 1.0e-3),
      std::string(snapshot.name) + " " + label +
          " derivative sign changed");
}

}  // namespace

int main() {
  wheel_leg::WheelAwareNmpcSolver solver;
  if (require(solver.ready(), "solver did not initialize")) return 1;
  const auto static_case = t0();
  const auto straight_case = t1();
  const auto static_solution = solve(solver, static_case.problem);
  const auto straight_solution = solve(solver, straight_case.problem);
  if (require(static_solution.ok() && straight_solution.ok(),
              "snapshot solve failed")) return 1;
  const double pitch_acceleration =
      predictedAcceleration(static_case.problem, static_solution, 10);
  const double longitudinal_acceleration =
      predictedAcceleration(straight_case.problem, straight_solution, 6);
  std::cout << "snapshot,base,predicted_acceleration\n"
            << static_case.name << ",pitch," << pitch_acceleration << '\n'
            << straight_case.name << ",longitudinal,"
            << longitudinal_acceleration << '\n';
  if (require(
          static_case.problem.state[4] * pitch_acceleration > 1.0e-6,
          "T0 snapshot did not reproduce reinforcing pitch acceleration") ||
      require(
          (straight_case.problem.state[6] - straight_case.problem.reference[6]) *
                  longitudinal_acceleration >
              1.0e-6,
          "T1 snapshot did not reproduce reinforcing longitudinal acceleration")) {
    return 1;
  }
  if (perturbation(solver, static_case, 4, 0.002, 10, "pitch", false) ||
      perturbation(solver, static_case, 10, 0.01, 10, "pitch_rate", false) ||
      perturbation(solver, straight_case, 0, 0.002, 6, "position_x", true) ||
      perturbation(solver, straight_case, 6, 0.01, 6, "velocity_x", true)) {
    return 1;
  }
  std::cout << "phase28 NMPC directionality oracle: PASS\n";
  return 0;
}
