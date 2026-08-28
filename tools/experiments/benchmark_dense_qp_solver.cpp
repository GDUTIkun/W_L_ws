#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "wheel_leg_core/dense_qp_solver.hpp"

namespace {
using Clock = std::chrono::steady_clock;
using wheel_leg::DenseQpSolver;

struct Problem {
  DenseQpSolver::Settings settings;
  int equality_rows{0};
  Eigen::MatrixXd h;
  Eigen::VectorXd g;
  Eigen::MatrixXd a;
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  std::optional<Eigen::VectorXd> oracle;
};
struct Summary {
  double p50_ms{0.0}, p99_ms{0.0}, max_ms{0.0}, mean_iterations{0.0};
  double max_primal{0.0}, max_dual{0.0}, max_stationarity{0.0};
  double max_bound{0.0}, max_equality{0.0};
};
struct OracleComparison {
  double max_scaled_x{0.0};
  double max_physical_torque_nm{0.0};
  double max_objective_gap{0.0};
};

template <typename Matrix>
void readValues(std::istream& input, Matrix& matrix) {
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      if (!(input >> matrix(row, column))) throw std::runtime_error("Truncated QP problem file");
    }
  }
}
double milliseconds(Clock::duration duration) {
  return std::chrono::duration<double, std::milli>(duration).count();
}
Problem readProblem(std::istream& input, int equality_rows,
                    DenseQpSolver::Settings settings, int variables,
                    int constraints, bool require_oracle) {
  if (variables != DenseQpSolver::kVariableCount || constraints < 0 ||
      constraints > DenseQpSolver::kMaxConstraintCount || equality_rows < 0 ||
      equality_rows > constraints) throw std::runtime_error("Invalid QP header");
  Problem problem{settings, equality_rows, Eigen::MatrixXd(variables, variables),
                  Eigen::VectorXd(variables), Eigen::MatrixXd(constraints, variables),
                  Eigen::VectorXd(constraints), Eigen::VectorXd(constraints), std::nullopt};
  readValues(input, problem.h); readValues(input, problem.g); readValues(input, problem.a);
  readValues(input, problem.lower); readValues(input, problem.upper);
  if (require_oracle) {
    std::string marker;
    if (!(input >> marker) || marker != "oracle") throw std::runtime_error("Corpus problem missing oracle vector");
    problem.oracle.emplace(variables); readValues(input, *problem.oracle);
  }
  return problem;
}
std::vector<Problem> readCorpus(const std::filesystem::path& path) {
  std::ifstream input(path); if (!input) throw std::runtime_error("Unable to open QP problem file");
  std::string first; if (!(input >> first)) throw std::runtime_error("Empty QP problem file");
  if (first == "DENSE_QP_CORPUS_V1") {
    int count = 0; if (!(input >> count) || count <= 0) throw std::runtime_error("Invalid corpus header");
    std::vector<Problem> corpus; corpus.reserve(static_cast<std::size_t>(count));
    for (int index = 0; index < count; ++index) {
      int variables = 0, constraints = 0, equality_rows = 0;
      double legacy_rho = 0.0, legacy_sigma = 0.0;
      DenseQpSolver::Settings settings;
      if (!(input >> variables >> constraints >> equality_rows >> legacy_rho >> legacy_sigma >>
            settings.absolute_tolerance >> settings.relative_tolerance >> settings.maximum_iterations))
        throw std::runtime_error("Truncated corpus header");
      corpus.push_back(readProblem(input, equality_rows, settings, variables, constraints, true));
    }
    return corpus;
  }
  int variables = 0; try { variables = std::stoi(first); } catch (const std::exception&) {
    throw std::runtime_error("Invalid QP header"); }
  int constraints = 0;
  double legacy_rho = 0.0, legacy_sigma = 0.0;
  DenseQpSolver::Settings settings;
  if (!(input >> constraints >> legacy_rho >> legacy_sigma >> settings.absolute_tolerance >>
        settings.relative_tolerance >> settings.maximum_iterations)) throw std::runtime_error("Invalid QP header");
  return {readProblem(input, 0, settings, variables, constraints, false)};
}
double boundViolation(const Problem& p, const Eigen::VectorXd& x) {
  if (p.a.rows() == 0) return 0.0;
  const Eigen::VectorXd ax = p.a * x;
  return std::max(0.0, std::max((p.lower - ax).maxCoeff(), (ax - p.upper).maxCoeff()));
}
double equalityResidual(const Problem& p, const Eigen::VectorXd& x) {
  return p.equality_rows == 0 ? 0.0 :
      (p.a.topRows(p.equality_rows) * x - p.lower.head(p.equality_rows)).cwiseAbs().maxCoeff();
}
Summary summarize(std::vector<double> times, const std::vector<DenseQpSolver::Result>& results,
                  const std::vector<const Problem*>& problems) {
  std::sort(times.begin(), times.end()); Summary s;
  s.p50_ms = times[times.size() / 2];
  s.p99_ms = times[static_cast<std::size_t>(std::floor(0.99 * (times.size() - 1)))];
  s.max_ms = times.back();
  for (std::size_t i = 0; i < results.size(); ++i) {
    const auto& r = results[i]; s.mean_iterations += r.iterations;
    s.max_primal = std::max(s.max_primal, r.primal_residual); s.max_dual = std::max(s.max_dual, r.dual_residual);
    s.max_stationarity = std::max(s.max_stationarity, r.stationarity_residual);
    s.max_bound = std::max(s.max_bound, boundViolation(*problems[i], r.x));
    s.max_equality = std::max(s.max_equality, equalityResidual(*problems[i], r.x));
  }
  s.mean_iterations /= static_cast<double>(results.size()); return s;
}
void writeSummary(std::ostream& o, const char* name, const Summary& s, bool comma) {
  o << "    \"" << name << "\": {\n"
    << "      \"p50_solve_time_ms\": " << s.p50_ms << ",\n"
    << "      \"p99_solve_time_ms\": " << s.p99_ms << ",\n"
    << "      \"max_solve_time_ms\": " << s.max_ms << ",\n"
    << "      \"mean_iterations\": " << s.mean_iterations << ",\n"
    << "      \"max_primal_residual\": " << s.max_primal << ",\n"
    << "      \"max_dual_residual\": " << s.max_dual << ",\n"
    << "      \"max_stationarity_residual\": " << s.max_stationarity << ",\n"
    << "      \"max_bound_violation\": " << s.max_bound << ",\n"
    << "      \"max_equality_residual\": " << s.max_equality << "\n    }" << (comma ? ",\n" : "\n");
}
DenseQpSolver::Result setupAndSolve(DenseQpSolver& solver, const Problem& p,
                                    DenseQpSolver::SetupMode setup_mode,
                                    DenseQpSolver::StartMode start_mode) {
  if (solver.setup(p.h, p.g, p.a, p.lower, p.upper, setup_mode) != DenseQpSolver::Status::kConverged)
    throw std::runtime_error("QP setup rejected benchmark problem");
  const auto r = solver.solve(start_mode);
  if (!r.converged()) throw std::runtime_error("Benchmark solve did not converge");
  return r;
}
std::optional<double> maxOracleError(
    const std::vector<DenseQpSolver::Result>& results,
    const std::vector<const Problem*>& problems) {
  double error = 0.0;
  for (std::size_t i = 0; i < results.size(); ++i) {
    if (!problems[i]->oracle) return std::nullopt;
    error = std::max(error,
        (results[i].x - *problems[i]->oracle).cwiseAbs().maxCoeff());
  }
  return error;
}
std::optional<OracleComparison> compareOracle(
    const std::vector<DenseQpSolver::Result>& results,
    const std::vector<const Problem*>& problems) {
  OracleComparison comparison;
  constexpr double torque_scale[6]{10.0, 10.0, 2.0, 10.0, 10.0, 2.0};
  for (std::size_t i = 0; i < results.size(); ++i) {
    if (!problems[i]->oracle) return std::nullopt;
    const auto& p = *problems[i];
    const Eigen::VectorXd difference = results[i].x - *p.oracle;
    comparison.max_scaled_x = std::max(
        comparison.max_scaled_x, difference.cwiseAbs().maxCoeff());
    for (int joint = 0; joint < 6; ++joint) {
      comparison.max_physical_torque_nm = std::max(
          comparison.max_physical_torque_nm,
          std::abs(torque_scale[joint] * difference[12 + joint]));
    }
    const auto objective = [&](const Eigen::VectorXd& x) {
      return 0.5 * x.dot(p.h * x) + p.g.dot(x);
    };
    comparison.max_objective_gap = std::max(
        comparison.max_objective_gap,
        std::abs(objective(results[i].x) - objective(*p.oracle)));
  }
  return comparison;
}
bool residualsPass(const Summary& summary) {
  return summary.max_bound <= 2.0e-7 && summary.max_equality <= 2.0e-7 &&
         summary.max_stationarity <= 2.0e-7;
}
}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 3 && argc != 4) throw std::runtime_error(
      "usage: benchmark_dense_qp_solver PROBLEM_OR_CORPUS OUTPUT [REPETITIONS=1000]\n"
      "legacy: vars rows rho sigma abs_tol rel_tol max_iter then H g A l u\n"
      "corpus: DENSE_QP_CORPUS_V1 count; each header is vars rows equality_rows rho sigma abs_tol rel_tol max_iter, followed by H g A l u and oracle x");
    const std::filesystem::path input_path(argv[1]), output_path(argv[2]);
    const int repetitions = argc == 4 ? std::stoi(argv[3]) : 1000;
    if (repetitions != 1000 || std::filesystem::exists(output_path))
      throw std::runtime_error("REPETITIONS must be 1000 and output must not exist");
    const auto corpus = readCorpus(input_path);
    const auto& settings = corpus.front().settings;
    for (const auto& problem : corpus) {
      if (problem.settings.absolute_tolerance != settings.absolute_tolerance ||
          problem.settings.relative_tolerance != settings.relative_tolerance ||
          problem.settings.maximum_iterations != settings.maximum_iterations) {
        throw std::runtime_error("All corpus problems must use identical solver settings");
      }
    }
    std::vector<double> cold_times, same_times, dynamic_times;
    std::vector<DenseQpSolver::Result> cold_results, same_results, dynamic_results;
    std::vector<const Problem*> cold_problems, same_problems, dynamic_problems;
    for (int i = 0; i < repetitions; ++i) {
      const Problem& p = corpus[static_cast<std::size_t>(i) % corpus.size()]; DenseQpSolver solver(p.settings);
      const auto start = Clock::now(); const auto r = setupAndSolve(solver, p, DenseQpSolver::SetupMode::kCold, DenseQpSolver::StartMode::kCold);
      cold_times.push_back(milliseconds(Clock::now() - start)); cold_results.push_back(r); cold_problems.push_back(&p);
    }
    const Problem& same = corpus.front(); DenseQpSolver same_solver(same.settings);
    setupAndSolve(same_solver, same, DenseQpSolver::SetupMode::kCold, DenseQpSolver::StartMode::kCold);
    for (int i = 0; i < repetitions; ++i) {
      const auto start = Clock::now(); const auto r = setupAndSolve(same_solver, same, DenseQpSolver::SetupMode::kWarm, DenseQpSolver::StartMode::kWarm);
      same_times.push_back(milliseconds(Clock::now() - start)); same_results.push_back(r); same_problems.push_back(&same);
    }
    DenseQpSolver dynamic_solver(corpus.front().settings);
    for (int i = 0; i < repetitions; ++i) {
      const Problem& p = corpus[static_cast<std::size_t>(i) % corpus.size()]; const auto start = Clock::now();
      const auto r = setupAndSolve(dynamic_solver, p, i == 0 ? DenseQpSolver::SetupMode::kCold : DenseQpSolver::SetupMode::kWarm,
                                   i == 0 ? DenseQpSolver::StartMode::kCold : DenseQpSolver::StartMode::kWarm);
      dynamic_times.push_back(milliseconds(Clock::now() - start)); dynamic_results.push_back(r); dynamic_problems.push_back(&p);
    }
    const auto cold = summarize(std::move(cold_times), cold_results, cold_problems);
    const auto same_warm = summarize(std::move(same_times), same_results, same_problems);
    const auto dynamic_warm = summarize(std::move(dynamic_times), dynamic_results, dynamic_problems);
    const auto cold_oracle = maxOracleError(cold_results, cold_problems);
    const auto same_oracle = maxOracleError(same_results, same_problems);
    const auto dynamic_oracle = maxOracleError(dynamic_results, dynamic_problems);
    const auto cold_comparison = compareOracle(cold_results, cold_problems);
    const auto same_comparison = compareOracle(same_results, same_problems);
    const auto dynamic_comparison = compareOracle(dynamic_results, dynamic_problems);
    const auto equivalent = [](const std::optional<OracleComparison>& value) {
      return value && (value->max_scaled_x <= 2.0e-6 ||
          (value->max_physical_torque_nm <= 5.0e-4 &&
           value->max_objective_gap <= 2.0e-6));
    };
    const bool oracle_pass = equivalent(cold_comparison) &&
        equivalent(same_comparison) && equivalent(dynamic_comparison);
    const bool deadline_pass = cold.max_ms <= 10.0 && dynamic_warm.max_ms <= 10.0;
    const bool pass = residualsPass(cold) && residualsPass(same_warm) &&
                      residualsPass(dynamic_warm) && oracle_pass && deadline_pass;
    std::ofstream output(output_path);
    if (!output) throw std::runtime_error("Unable to create benchmark output");
    output << std::setprecision(17) << "{\n  \"schema_version\": 3,\n"
      << "  \"solver\": \"ProxSuite ProxQP 0.7.3 dense PrimalDualLDLT\",\n"
      << "  \"repetitions\": " << repetitions << ",\n  \"corpus_problem_count\": " << corpus.size() << ",\n"
      << "  \"cold_oracle_max_abs_error\": ";
    if (cold_oracle) output << *cold_oracle;
    else output << "null";
    output << ",\n  \"repeated_same_warm_oracle_max_abs_error\": ";
    if (same_oracle) output << *same_oracle;
    else output << "null";
    output << ",\n  \"cycling_dynamic_warm_oracle_max_abs_error\": ";
    if (dynamic_oracle) output << *dynamic_oracle;
    else output << "null";
    output << ",\n  \"cold_oracle_max_physical_torque_error_nm\": "
           << (cold_comparison ? cold_comparison->max_physical_torque_nm : -1.0)
           << ",\n  \"cold_oracle_max_objective_gap\": "
           << (cold_comparison ? cold_comparison->max_objective_gap : -1.0)
           << ",\n  \"cycling_dynamic_warm_oracle_max_physical_torque_error_nm\": "
           << (dynamic_comparison ? dynamic_comparison->max_physical_torque_nm : -1.0)
           << ",\n  \"cycling_dynamic_warm_oracle_max_objective_gap\": "
           << (dynamic_comparison ? dynamic_comparison->max_objective_gap : -1.0);
    output << ",\n  \"modes\": {\n";
    writeSummary(output, "cold", cold, true); writeSummary(output, "repeated_same_warm", same_warm, true);
    writeSummary(output, "cycling_dynamic_warm", dynamic_warm, false);
    output << "  },\n  \"gates\": {\n"
           << "    \"bound_equality_stationarity\": "
           << ((residualsPass(cold) && residualsPass(same_warm) &&
                residualsPass(dynamic_warm)) ? "true" : "false") << ",\n"
           << "    \"oracle_or_physical_output_objective_equivalence\": "
           << (oracle_pass ? "true" : "false") << ",\n"
           << "    \"cold_dynamic_max_setup_solve_ms\": "
           << (deadline_pass ? "true" : "false") << "\n  },\n"
           << "  \"pass\": " << (pass ? "true" : "false") << "\n}\n";
    return pass ? 0 : 1;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 2; }
}
