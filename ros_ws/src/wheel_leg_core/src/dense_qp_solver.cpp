#include "wheel_leg_core/dense_qp_solver.hpp"

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>

namespace wheel_leg {
namespace {

constexpr double kSymmetryTolerance = 1.0e-10;
constexpr double kPsdTolerance = 1.0e-10;

}  // namespace

DenseQpSolver::DenseQpSolver() = default;

DenseQpSolver::DenseQpSolver(Settings settings) : settings_(settings) {}

bool DenseQpSolver::validSettings() const {
  return std::isfinite(settings_.rho) && settings_.rho > 0.0 &&
         std::isfinite(settings_.sigma) && settings_.sigma > 0.0 &&
         std::isfinite(settings_.relaxation) && settings_.relaxation > 0.0 &&
         settings_.relaxation < 2.0 &&
         std::isfinite(settings_.absolute_tolerance) &&
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
  const bool preserve_warm_start =
      setup_mode == SetupMode::kWarm && ready_ && constraint_count_ == a.rows();
  ready_ = false;
  constraint_count_ = 0;
  if (!preserve_warm_start) {
    reset();
  }
  if (!validSettings() || h.rows() != kVariableCount ||
      h.cols() != kVariableCount || g.size() != kVariableCount ||
      a.cols() != kVariableCount || a.rows() < 0 ||
      a.rows() > kMaxConstraintCount || lower.size() != a.rows() ||
      upper.size() != a.rows() || !h.allFinite() || !g.allFinite() ||
      !a.allFinite() || !lower.allFinite() || !upper.allFinite()) {
    return Status::kInvalidInput;
  }
  if ((h - h.transpose()).cwiseAbs().maxCoeff() >
      kSymmetryTolerance * std::max(1.0, h.cwiseAbs().maxCoeff())) {
    return Status::kInvalidInput;
  }
  if ((lower.array() > upper.array()).any()) {
    return Status::kInvalidInput;
  }

  h_ = h;
  g_ = g;
  constraint_count_ = a.rows();
  if (constraint_count_ > 0) {
    a_.topRows(constraint_count_) = a;
    lower_.head(constraint_count_) = lower;
    upper_.head(constraint_count_) = upper;
  }
  if (!isPositiveSemidefinite()) {
    return Status::kInvalidInput;
  }

  system_ = h_;
  if (constraint_count_ > 0) {
    system_.diagonal().array() += settings_.sigma;
    system_.noalias() += settings_.rho *
                         a_.topRows(constraint_count_).transpose() *
                             a_.topRows(constraint_count_);
  }
  factorization_.compute(system_);
  if (factorization_.info() != Eigen::Success ||
      !factorization_.vectorD().allFinite() ||
      (factorization_.vectorD().array() <= 0.0).any()) {
    return Status::kFactorizationFailure;
  }
  if (preserve_warm_start && constraint_count_ > 0) {
    ax_.head(constraint_count_).noalias() =
        a_.topRows(constraint_count_) * x_;
    z_.head(constraint_count_) =
        (ax_.head(constraint_count_) + y_.head(constraint_count_))
            .cwiseMax(lower_.head(constraint_count_))
            .cwiseMin(upper_.head(constraint_count_));
    z_previous_.head(constraint_count_) = z_.head(constraint_count_);
  }
  ready_ = true;
  return Status::kConverged;
}

void DenseQpSolver::reset() {
  x_.setZero();
  rhs_.setZero();
  z_.setZero();
  z_previous_.setZero();
  y_.setZero();
  ax_.setZero();
  residual_.setZero();
  relaxed_.setZero();
  dual_work_.setZero();
}

DenseQpSolver::Result DenseQpSolver::rejected(Status status) const {
  Result result;
  result.status = status;
  return result;
}

DenseQpSolver::Result DenseQpSolver::solve(StartMode start_mode) {
  if (!ready_) {
    return rejected(Status::kInvalidInput);
  }
  if (start_mode == StartMode::kCold) {
    reset();
  }

  Result result;
  if (constraint_count_ == 0) {
    rhs_ = -g_;
    x_ = factorization_.solve(rhs_);
    if (!x_.allFinite()) {
      return rejected(Status::kNonFinite);
    }
    dual_work_.noalias() = h_ * x_ + g_;
    const double stationarity = dual_work_.norm();
    const double stationarity_tolerance =
        std::sqrt(static_cast<double>(kVariableCount)) *
            settings_.absolute_tolerance +
        settings_.relative_tolerance *
            std::max((h_ * x_).norm(), g_.norm());
    result.iterations = 1;
    result.stationarity_residual = stationarity;
    if (!std::isfinite(stationarity) ||
        !std::isfinite(stationarity_tolerance)) {
      return rejected(Status::kNonFinite);
    }
    if (stationarity > stationarity_tolerance) {
      result.status = Status::kMaximumIterations;
      return result;
    }
    result.status = Status::kConverged;
    result.x = x_;
    return result;
  }
  for (int iteration = 1; iteration <= settings_.maximum_iterations;
       ++iteration) {
    rhs_ = -g_ + settings_.sigma * x_;
    if (constraint_count_ > 0) {
      rhs_.noalias() += settings_.rho *
                         a_.topRows(constraint_count_).transpose() *
                             (z_.head(constraint_count_) -
                              y_.head(constraint_count_));
    }
    x_ = factorization_.solve(rhs_);
    if (!x_.allFinite()) {
      return rejected(Status::kNonFinite);
    }

    ax_.head(constraint_count_).noalias() =
        a_.topRows(constraint_count_) * x_;
    z_previous_.head(constraint_count_) = z_.head(constraint_count_);
    relaxed_.head(constraint_count_) =
        settings_.relaxation * ax_.head(constraint_count_) +
        (1.0 - settings_.relaxation) * z_previous_.head(constraint_count_);
    z_.head(constraint_count_) =
        (relaxed_.head(constraint_count_) + y_.head(constraint_count_))
            .cwiseMax(lower_.head(constraint_count_))
            .cwiseMin(upper_.head(constraint_count_));
    residual_.head(constraint_count_) =
        ax_.head(constraint_count_) - z_.head(constraint_count_);
    y_.head(constraint_count_) +=
        relaxed_.head(constraint_count_) - z_.head(constraint_count_);
    if (!ax_.head(constraint_count_).allFinite() ||
        !z_.head(constraint_count_).allFinite() ||
        !y_.head(constraint_count_).allFinite()) {
      return rejected(Status::kNonFinite);
    }

    const double primal = residual_.head(constraint_count_).norm();
    dual_work_.noalias() = settings_.rho *
                           a_.topRows(constraint_count_).transpose() *
                               (z_.head(constraint_count_) -
                                z_previous_.head(constraint_count_));
    const double dual = dual_work_.norm();
    const double primal_tolerance =
        std::sqrt(static_cast<double>(constraint_count_)) *
            settings_.absolute_tolerance +
        settings_.relative_tolerance *
            std::max(ax_.head(constraint_count_).norm(),
                     z_.head(constraint_count_).norm());
    const double dual_tolerance =
        std::sqrt(static_cast<double>(kVariableCount)) *
            settings_.absolute_tolerance +
        settings_.relative_tolerance *
            (settings_.rho *
             (a_.topRows(constraint_count_).transpose() *
             y_.head(constraint_count_)).norm());
    dual_work_.noalias() = h_ * x_ + g_ + settings_.rho *
                           a_.topRows(constraint_count_).transpose() *
                               y_.head(constraint_count_);
    const double stationarity = dual_work_.norm();
    if (!std::isfinite(primal) || !std::isfinite(dual) ||
        !std::isfinite(stationarity) ||
        !std::isfinite(primal_tolerance) || !std::isfinite(dual_tolerance)) {
      return rejected(Status::kNonFinite);
    }
    if (primal <= primal_tolerance && dual <= dual_tolerance) {
      result.status = Status::kConverged;
      result.iterations = iteration;
      result.primal_residual = primal;
      result.dual_residual = dual;
      result.stationarity_residual = stationarity;
      result.x = x_;
      return result;
    }
    result.iterations = iteration;
    result.primal_residual = primal;
    result.dual_residual = dual;
    result.stationarity_residual = stationarity;
  }
  result.status = Status::kMaximumIterations;
  return result;
}

}  // namespace wheel_leg
