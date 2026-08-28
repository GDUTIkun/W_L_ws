#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <proxsuite/proxqp/dense/dense.hpp>

namespace {
using Clock = std::chrono::steady_clock;
using Qp = proxsuite::proxqp::dense::QP<double>;

struct Problem {
  Eigen::MatrixXd h;
  Eigen::VectorXd g;
  Eigen::MatrixXd a;
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
  Eigen::VectorXd oracle;
};

struct SplitProblem {
  Eigen::MatrixXd a;
  Eigen::VectorXd b;
  Eigen::MatrixXd c;
  Eigen::VectorXd lower;
  Eigen::VectorXd upper;
};

struct Sample {
  double milliseconds{0.0};
  int iterations{0};
  double primal{0.0};
  double dual{0.0};
  double stationarity{0.0};
  double bound{0.0};
  double equality{0.0};
  double scaled_x{0.0};
  double physical_torque_nm{0.0};
  double objective_gap{0.0};
};

template <typename Matrix>
void readValues(std::istream& input, Matrix& matrix) {
  for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
    for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
      if (!(input >> matrix(row, column))) {
        throw std::runtime_error("truncated corpus");
      }
    }
  }
}

std::vector<Problem> readCorpus(const std::string& path) {
  std::ifstream input(path);
  std::string marker;
  int count = 0;
  if (!input || !(input >> marker >> count) || marker != "DENSE_QP_CORPUS_V1" ||
      count <= 0) {
    throw std::runtime_error("invalid corpus header");
  }
  std::vector<Problem> problems;
  problems.reserve(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index) {
    int variables = 0;
    int constraints = 0;
    int equality_rows = 0;
    double ignored = 0.0;
    int ignored_iterations = 0;
    if (!(input >> variables >> constraints >> equality_rows >> ignored >> ignored >>
          ignored >> ignored >> ignored_iterations) ||
        variables != 42 || constraints < 0 || constraints > 128 ||
        equality_rows < 0 || equality_rows > constraints) {
      throw std::runtime_error("invalid problem header");
    }
    Problem problem{Eigen::MatrixXd(variables, variables),
                    Eigen::VectorXd(variables),
                    Eigen::MatrixXd(constraints, variables),
                    Eigen::VectorXd(constraints),
                    Eigen::VectorXd(constraints),
                    Eigen::VectorXd(variables)};
    readValues(input, problem.h);
    readValues(input, problem.g);
    readValues(input, problem.a);
    readValues(input, problem.lower);
    readValues(input, problem.upper);
    if (!(input >> marker) || marker != "oracle") {
      throw std::runtime_error("missing oracle");
    }
    readValues(input, problem.oracle);
    problems.push_back(std::move(problem));
  }
  return problems;
}

SplitProblem split(const Problem& problem) {
  int equality_count = 0;
  for (Eigen::Index row = 0; row < problem.a.rows(); ++row) {
    equality_count += problem.lower[row] == problem.upper[row] ? 1 : 0;
  }
  SplitProblem result{Eigen::MatrixXd(equality_count, problem.a.cols()),
                      Eigen::VectorXd(equality_count),
                      Eigen::MatrixXd(problem.a.rows() - equality_count,
                                      problem.a.cols()),
                      Eigen::VectorXd(problem.a.rows() - equality_count),
                      Eigen::VectorXd(problem.a.rows() - equality_count)};
  int equality = 0;
  int inequality = 0;
  for (Eigen::Index row = 0; row < problem.a.rows(); ++row) {
    if (problem.lower[row] == problem.upper[row]) {
      result.a.row(equality) = problem.a.row(row);
      result.b[equality++] = problem.lower[row];
    } else {
      result.c.row(inequality) = problem.a.row(row);
      result.lower[inequality] = problem.lower[row];
      result.upper[inequality++] = problem.upper[row];
    }
  }
  return result;
}

void configure(Qp& qp, proxsuite::proxqp::InitialGuessStatus initial_guess) {
  qp.settings.eps_abs = 1.0e-8;
  qp.settings.eps_rel = 1.0e-8;
  qp.settings.max_iter = 10000;
  qp.settings.verbose = false;
  qp.settings.primal_infeasibility_solving = false;
  qp.settings.initial_guess = initial_guess;
}

Sample sample(const Problem& problem, const SplitProblem& split_problem,
              const Qp& qp, Clock::time_point started) {
  if (qp.results.info.status !=
      proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED) {
    throw std::runtime_error("ProxQP did not solve corpus problem");
  }
  const Eigen::VectorXd& x = qp.results.x;
  const Eigen::VectorXd ax = problem.a * x;
  const Eigen::VectorXd stationarity =
      problem.h * x + problem.g + split_problem.a.transpose() * qp.results.y +
      split_problem.c.transpose() * qp.results.z;
  Sample result;
  result.milliseconds =
      std::chrono::duration<double, std::milli>(Clock::now() - started).count();
  result.iterations = static_cast<int>(qp.results.info.iter);
  result.primal = qp.results.info.pri_res;
  result.dual = qp.results.info.dua_res;
  result.stationarity = stationarity.lpNorm<Eigen::Infinity>();
  result.bound = std::max(
      0.0, std::max((problem.lower - ax).maxCoeff(),
                    (ax - problem.upper).maxCoeff()));
  result.equality = split_problem.a.rows() == 0
                        ? 0.0
                        : (split_problem.a * x - split_problem.b)
                              .cwiseAbs()
                              .maxCoeff();
  result.scaled_x = (x - problem.oracle).cwiseAbs().maxCoeff();
  constexpr double torque_scale[6]{10.0, 10.0, 2.0, 10.0, 10.0, 2.0};
  for (int joint = 0; joint < 6; ++joint) {
    result.physical_torque_nm = std::max(
        result.physical_torque_nm,
        std::abs(torque_scale[joint] * (x[12 + joint] - problem.oracle[12 + joint])));
  }
  const auto objective = [&](const Eigen::VectorXd& candidate) {
    return 0.5 * candidate.dot(problem.h * candidate) +
           problem.g.dot(candidate);
  };
  result.objective_gap = std::abs(objective(x) - objective(problem.oracle));
  return result;
}

void updateMax(Sample& maximum, const Sample& value) {
  maximum.milliseconds = std::max(maximum.milliseconds, value.milliseconds);
  maximum.iterations = std::max(maximum.iterations, value.iterations);
  maximum.primal = std::max(maximum.primal, value.primal);
  maximum.dual = std::max(maximum.dual, value.dual);
  maximum.stationarity = std::max(maximum.stationarity, value.stationarity);
  maximum.bound = std::max(maximum.bound, value.bound);
  maximum.equality = std::max(maximum.equality, value.equality);
  maximum.scaled_x = std::max(maximum.scaled_x, value.scaled_x);
  maximum.physical_torque_nm =
      std::max(maximum.physical_torque_nm, value.physical_torque_nm);
  maximum.objective_gap = std::max(maximum.objective_gap, value.objective_gap);
}

Sample cold(const Problem& problem) {
  const auto started = Clock::now();
  const SplitProblem split_problem = split(problem);
  Qp qp(42, split_problem.a.rows(), split_problem.c.rows(), false,
        proxsuite::proxqp::DenseBackend::PrimalDualLDLT);
  configure(qp, proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS);
  qp.init(problem.h, problem.g, split_problem.a, split_problem.b,
          split_problem.c, split_problem.lower, split_problem.upper, true);
  qp.solve();
  return sample(problem, split_problem, qp, started);
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc != 2) {
      throw std::runtime_error("usage: profile_proxqp_solver CORPUS");
    }
    const std::vector<Problem> corpus = readCorpus(argv[1]);
    Sample cold_max;
    for (int repetition = 0; repetition < 1000; ++repetition) {
      updateMax(cold_max, cold(corpus[static_cast<std::size_t>(repetition) %
                                      corpus.size()]));
    }

    const SplitProblem initial = split(corpus.front());
    Qp warm_qp(42, initial.a.rows(), initial.c.rows(), false,
               proxsuite::proxqp::DenseBackend::PrimalDualLDLT);
    configure(warm_qp,
              proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT);
    warm_qp.init(corpus.front().h, corpus.front().g, initial.a, initial.b,
                 initial.c, initial.lower, initial.upper, true);
    warm_qp.solve();
    Sample warm_max;
    for (int repetition = 0; repetition < 1000; ++repetition) {
      const Problem& problem = corpus[static_cast<std::size_t>(repetition) % corpus.size()];
      const auto started = Clock::now();
      const SplitProblem current = split(problem);
      warm_qp.update(problem.h, problem.g, current.a, current.b, current.c,
                     current.lower, current.upper, false);
      warm_qp.solve();
      updateMax(warm_max, sample(problem, current, warm_qp, started));
    }

    const bool pass = cold_max.bound <= 2.0e-7 && cold_max.equality <= 2.0e-7 &&
                      cold_max.stationarity <= 2.0e-7 &&
                      warm_max.bound <= 2.0e-7 && warm_max.equality <= 2.0e-7 &&
                      warm_max.stationarity <= 2.0e-7 &&
                      cold_max.physical_torque_nm <= 5.0e-4 &&
                      warm_max.physical_torque_nm <= 5.0e-4 &&
                      cold_max.objective_gap <= 2.0e-6 &&
                      warm_max.objective_gap <= 2.0e-6 &&
                      cold_max.milliseconds <= 10.0 && warm_max.milliseconds <= 10.0;
    std::cout << std::setprecision(17)
              << "cold_max_ms=" << cold_max.milliseconds << '\n'
              << "warm_dynamic_max_ms=" << warm_max.milliseconds << '\n'
              << "cold_max_iterations=" << cold_max.iterations << '\n'
              << "warm_max_iterations=" << warm_max.iterations << '\n'
              << "max_primal=" << std::max(cold_max.primal, warm_max.primal) << '\n'
              << "max_dual=" << std::max(cold_max.dual, warm_max.dual) << '\n'
              << "max_stationarity=" << std::max(cold_max.stationarity, warm_max.stationarity) << '\n'
              << "max_bound=" << std::max(cold_max.bound, warm_max.bound) << '\n'
              << "max_equality=" << std::max(cold_max.equality, warm_max.equality) << '\n'
              << "max_scaled_x=" << std::max(cold_max.scaled_x, warm_max.scaled_x) << '\n'
              << "max_physical_torque_nm=" << std::max(cold_max.physical_torque_nm, warm_max.physical_torque_nm) << '\n'
              << "max_objective_gap=" << std::max(cold_max.objective_gap, warm_max.objective_gap) << '\n'
              << "pass=" << (pass ? "true" : "false") << '\n';
    return pass ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
