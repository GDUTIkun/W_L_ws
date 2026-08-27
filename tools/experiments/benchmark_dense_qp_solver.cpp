#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

#include "wheel_leg_core/dense_qp_solver.hpp"

namespace {

using Clock = std::chrono::steady_clock;
using wheel_leg::DenseQpSolver;

template <typename Matrix>
void readValues(std::istream &input, Matrix &matrix) {
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      if (!(input >> matrix(row, column))) {
        throw std::runtime_error("Truncated QP problem file");
      }
    }
  }
}

double milliseconds(Clock::duration duration) {
  return std::chrono::duration<double, std::milli>(duration).count();
}

}  // namespace

int main(int argc, char **argv) {
  try {
    if (argc != 4) {
      throw std::runtime_error("usage: benchmark_dense_qp_solver PROBLEM OUTPUT REPETITIONS");
    }
    const std::filesystem::path problem_path(argv[1]);
    const std::filesystem::path output_path(argv[2]);
    const int repetitions = std::stoi(argv[3]);
    if (repetitions <= 0 || std::filesystem::exists(output_path)) {
      throw std::runtime_error("Invalid repetitions or existing output path");
    }
    std::ifstream input(problem_path);
    int variables = 0;
    int constraints = 0;
    DenseQpSolver::Settings settings;
    if (!(input >> variables >> constraints >> settings.rho >> settings.sigma >>
          settings.absolute_tolerance >> settings.relative_tolerance >>
          settings.maximum_iterations) || variables != DenseQpSolver::kVariableCount ||
        constraints < 0 || constraints > DenseQpSolver::kMaxConstraintCount) {
      throw std::runtime_error("Invalid QP header");
    }
    Eigen::MatrixXd h(variables, variables);
    Eigen::VectorXd g(variables);
    Eigen::MatrixXd a(constraints, variables);
    Eigen::VectorXd lower(constraints);
    Eigen::VectorXd upper(constraints);
    readValues(input, h);
    readValues(input, g);
    readValues(input, a);
    readValues(input, lower);
    readValues(input, upper);

    DenseQpSolver solver(settings);
    const auto setup_start = Clock::now();
    const auto setup_status = solver.setup(h, g, a, lower, upper);
    const double setup_ms = milliseconds(Clock::now() - setup_start);
    if (setup_status != DenseQpSolver::Status::kConverged) {
      throw std::runtime_error("QP setup rejected benchmark problem");
    }
    std::vector<double> cold_times;
    cold_times.reserve(static_cast<std::size_t>(repetitions));
    DenseQpSolver::Result cold;
    for (int index = 0; index < repetitions; ++index) {
      const auto start = Clock::now();
      cold = solver.solve(DenseQpSolver::StartMode::kCold);
      cold_times.push_back(milliseconds(Clock::now() - start));
      if (!cold.converged()) {
        throw std::runtime_error("Cold benchmark solve did not converge");
      }
    }
    const auto warm_start = Clock::now();
    const auto warm = solver.solve(DenseQpSolver::StartMode::kWarm);
    const double warm_ms = milliseconds(Clock::now() - warm_start);
    if (!warm.converged()) {
      throw std::runtime_error("Warm benchmark solve did not converge");
    }
    const Eigen::VectorXd ax = a * cold.x;
    const double bound_violation = std::max(
        0.0, std::max((lower - ax).maxCoeff(), (ax - upper).maxCoeff()));
    const double equality_residual =
        constraints < 24 ? std::numeric_limits<double>::infinity() :
        (a.topRows(24) * cold.x - lower.head(24)).cwiseAbs().maxCoeff();
    const double warm_difference = (warm.x - cold.x).cwiseAbs().maxCoeff();
    std::sort(cold_times.begin(), cold_times.end());
    const std::size_t p99_index = static_cast<std::size_t>(
        std::floor(0.99 * static_cast<double>(cold_times.size() - 1)));
    const bool pass = equality_residual <= 2e-7 && bound_violation <= 2e-7 &&
                      cold.stationarity_residual <= 2e-6 &&
                      warm_difference <= 2e-6 && cold_times.back() <= 10.0;

    std::ofstream output(output_path);
    output << std::setprecision(17)
           << "{\n"
           << "  \"schema_version\": 1,\n"
           << "  \"repetitions\": " << repetitions << ",\n"
           << "  \"setup_time_ms\": " << setup_ms << ",\n"
           << "  \"cold_iterations\": " << cold.iterations << ",\n"
           << "  \"warm_iterations\": " << warm.iterations << ",\n"
           << "  \"cold_p99_solve_time_ms\": " << cold_times[p99_index] << ",\n"
           << "  \"cold_max_solve_time_ms\": " << cold_times.back() << ",\n"
           << "  \"warm_solve_time_ms\": " << warm_ms << ",\n"
           << "  \"equality_residual\": " << equality_residual << ",\n"
           << "  \"bound_violation\": " << bound_violation << ",\n"
           << "  \"stationarity_residual\": " << cold.stationarity_residual << ",\n"
           << "  \"warm_cold_max_difference\": " << warm_difference << ",\n"
           << "  \"pass\": " << (pass ? "true" : "false") << "\n"
           << "}\n";
    return pass ? 0 : 1;
  } catch (const std::exception &error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
