#include "wheel_leg_core/dense_qp_solver.hpp"

#include <Eigen/Eigenvalues>
#include <proxsuite/proxqp/dense/dense.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <utility>

namespace wheel_leg {
namespace {

constexpr double kSymmetryTolerance = 1.0e-10;
constexpr double kPsdTolerance = 1.0e-10;
using Qp = proxsuite::proxqp::dense::QP<double>;

}  // namespace

class DenseQpSolver::Impl {
 public:
  std::unique_ptr<Qp> qp;
  Eigen::MatrixXd equality_matrix;
  Eigen::VectorXd equality_target;
  Eigen::MatrixXd inequality_matrix;
  Eigen::VectorXd inequality_lower;
  Eigen::VectorXd inequality_upper;
};

DenseQpSolver::DenseQpSolver() : impl_(std::make_unique<Impl>()) {}

DenseQpSolver::DenseQpSolver(Settings settings)
    : settings_(settings), impl_(std::make_unique<Impl>()) {}

DenseQpSolver::~DenseQpSolver() = default;
DenseQpSolver::DenseQpSolver(DenseQpSolver&&) noexcept = default;
DenseQpSolver& DenseQpSolver::operator=(DenseQpSolver&&) noexcept = default;

bool DenseQpSolver::validSettings() const {
  return std::isfinite(settings_.absolute_tolerance) &&
         settings_.absolute_tolerance >= 0.0 &&
         std::isfinite(settings_.relative_tolerance) &&
         settings_.relative_tolerance >= 0.0 && settings_.maximum_iterations > 0;
}

bool DenseQpSolver::isPositiveSemidefinite() const {
  Eigen::SelfAdjointEigenSolver<Matrix> eigen_solver(h_);
  if (eigen_solver.info() != Eigen::Success) {
    return false;
  }
  const double scale = std::max(1.0, h_.cwiseAbs().maxCoeff());
  return eigen_solver.eigenvalues().minCoeff() >= -kPsdTolerance * scale;
}

DenseQpSolver::Status DenseQpSolver::setup(
    const Eigen::Ref<const Eigen::MatrixXd>& h,
    const Eigen::Ref<const Eigen::VectorXd>& g,
    const Eigen::Ref<const Eigen::MatrixXd>& a,
    const Eigen::Ref<const Eigen::VectorXd>& lower,
    const Eigen::Ref<const Eigen::VectorXd>& upper, SetupMode setup_mode) {
  if (!validSettings() || h.rows() != kVariableCount ||
      h.cols() != kVariableCount || g.size() != kVariableCount ||
      a.cols() != kVariableCount || a.rows() < 0 ||
      a.rows() > kMaxConstraintCount || lower.size() != a.rows() ||
      upper.size() != a.rows() || !h.allFinite() || !g.allFinite() ||
      !a.allFinite() || !lower.allFinite() || !upper.allFinite() ||
      (lower.array() > upper.array()).any() ||
      (h - h.transpose()).cwiseAbs().maxCoeff() >
          kSymmetryTolerance * std::max(1.0, h.cwiseAbs().maxCoeff())) {
    reset();
    return Status::kInvalidInput;
  }

  std::array<bool, kMaxConstraintCount> equality_mask{};
  int equality_count = 0;
  for (Eigen::Index row = 0; row < a.rows(); ++row) {
    const bool equality = lower[row] == upper[row];
    equality_mask[static_cast<std::size_t>(row)] = equality;
    equality_count += equality ? 1 : 0;
  }
  const bool compatible_warm_start =
      setup_mode == SetupMode::kWarm && ready_ && previous_solve_succeeded_ &&
      constraint_count_ == a.rows() && equality_count_ == equality_count &&
      equality_mask_ == equality_mask;

  h_ = h;
  g_ = g;
  if (!isPositiveSemidefinite()) {
    reset();
    return Status::kInvalidInput;
  }
  if (!compatible_warm_start) {
    reset();
    h_ = h;
    g_ = g;
  }
  constraint_count_ = static_cast<int>(a.rows());
  equality_count_ = equality_count;
  equality_mask_ = equality_mask;
  const int inequality_count = constraint_count_ - equality_count_;
  impl_->equality_matrix.resize(equality_count_, kVariableCount);
  impl_->equality_target.resize(equality_count_);
  impl_->inequality_matrix.resize(inequality_count, kVariableCount);
  impl_->inequality_lower.resize(inequality_count);
  impl_->inequality_upper.resize(inequality_count);
  int equality_row = 0;
  int inequality_row = 0;
  for (int row = 0; row < constraint_count_; ++row) {
    if (equality_mask_[static_cast<std::size_t>(row)]) {
      impl_->equality_matrix.row(equality_row) = a.row(row);
      impl_->equality_target[equality_row++] = lower[row];
    } else {
      impl_->inequality_matrix.row(inequality_row) = a.row(row);
      impl_->inequality_lower[inequality_row] = lower[row];
      impl_->inequality_upper[inequality_row++] = upper[row];
    }
  }

  try {
    if (!compatible_warm_start) {
      impl_->qp = std::make_unique<Qp>(
          kVariableCount, equality_count_, inequality_count, false,
          proxsuite::proxqp::DenseBackend::PrimalDualLDLT);
      impl_->qp->settings.eps_abs = settings_.absolute_tolerance;
      impl_->qp->settings.eps_rel = settings_.relative_tolerance;
      impl_->qp->settings.max_iter = settings_.maximum_iterations;
      impl_->qp->settings.verbose = false;
      impl_->qp->settings.primal_infeasibility_solving = false;
      impl_->qp->settings.initial_guess =
          proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;
      impl_->qp->init(h_, g_, impl_->equality_matrix,
                      impl_->equality_target, impl_->inequality_matrix,
                      impl_->inequality_lower, impl_->inequality_upper, true);
    } else {
      impl_->qp->settings.initial_guess =
          proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
      impl_->qp->update(h_, g_, impl_->equality_matrix,
                        impl_->equality_target, impl_->inequality_matrix,
                        impl_->inequality_lower, impl_->inequality_upper, false);
    }
  } catch (const std::exception&) {
    reset();
    return Status::kFactorizationFailure;
  }
  ready_ = true;
  previous_solve_succeeded_ = compatible_warm_start;
  return Status::kConverged;
}

void DenseQpSolver::reset() {
  constraint_count_ = 0;
  equality_count_ = 0;
  ready_ = false;
  previous_solve_succeeded_ = false;
  equality_mask_.fill(false);
  if (impl_) {
    impl_->qp.reset();
    impl_->equality_matrix.resize(0, kVariableCount);
    impl_->equality_target.resize(0);
    impl_->inequality_matrix.resize(0, kVariableCount);
    impl_->inequality_lower.resize(0);
    impl_->inequality_upper.resize(0);
  }
}

DenseQpSolver::Result DenseQpSolver::rejected(Status status) const {
  Result result;
  result.status = status;
  return result;
}

DenseQpSolver::Result DenseQpSolver::solve(StartMode start_mode) {
  if (!ready_ || !impl_ || !impl_->qp) {
    return rejected(Status::kInvalidInput);
  }
  const bool warm_start =
      start_mode == StartMode::kWarm && previous_solve_succeeded_;
  impl_->qp->settings.initial_guess =
      warm_start
          ? proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT
          : proxsuite::proxqp::InitialGuessStatus::NO_INITIAL_GUESS;
  try {
    impl_->qp->solve();
  } catch (const std::exception&) {
    previous_solve_succeeded_ = false;
    return rejected(Status::kFactorizationFailure);
  }

  const auto& info = impl_->qp->results.info;
  Result result;
  result.iterations = static_cast<int>(info.iter);
  result.primal_residual = info.pri_res;
  result.dual_residual = info.dua_res;
  const Eigen::VectorXd& candidate = impl_->qp->results.x;
  if (candidate.size() != kVariableCount || !candidate.allFinite() ||
      !std::isfinite(result.primal_residual) ||
      !std::isfinite(result.dual_residual)) {
    previous_solve_succeeded_ = false;
    return rejected(Status::kNonFinite);
  }
  const Eigen::VectorXd stationarity =
      h_ * candidate + g_ +
      impl_->equality_matrix.transpose() * impl_->qp->results.y +
      impl_->inequality_matrix.transpose() * impl_->qp->results.z;
  if (!stationarity.allFinite()) {
    previous_solve_succeeded_ = false;
    return rejected(Status::kNonFinite);
  }
  result.stationarity_residual = stationarity.lpNorm<Eigen::Infinity>();

  switch (info.status) {
    case proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED:
      result.status = Status::kConverged;
      result.x = candidate;
      previous_solve_succeeded_ = true;
      return result;
    case proxsuite::proxqp::QPSolverOutput::PROXQP_MAX_ITER_REACHED:
      result.status = Status::kMaximumIterations;
      break;
    case proxsuite::proxqp::QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE:
    case proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE:
      result.status = Status::kPrimalInfeasible;
      break;
    case proxsuite::proxqp::QPSolverOutput::PROXQP_DUAL_INFEASIBLE:
      result.status = Status::kDualInfeasible;
      break;
    case proxsuite::proxqp::QPSolverOutput::PROXQP_NOT_RUN:
      result.status = Status::kFactorizationFailure;
      break;
  }
  previous_solve_succeeded_ = false;
  result.x.setZero();
  return result;
}

}  // namespace wheel_leg
