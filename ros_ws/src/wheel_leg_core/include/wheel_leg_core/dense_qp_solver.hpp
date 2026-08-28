#pragma once

#include <array>
#include <memory>

#include <Eigen/Core>

namespace wheel_leg {

// Fixed-capacity adapter for the Phase 22 ProxQP bound-form QP.
class DenseQpSolver {
 public:
  static constexpr int kVariableCount = 42;
  static constexpr int kMaxConstraintCount = 128;

  using Vector = Eigen::Matrix<double, kVariableCount, 1>;
  using Matrix = Eigen::Matrix<double, kVariableCount, kVariableCount>;
  using ConstraintMatrix =
      Eigen::Matrix<double, kMaxConstraintCount, kVariableCount>;
  using ConstraintVector = Eigen::Matrix<double, kMaxConstraintCount, 1>;

  struct Settings {
    double absolute_tolerance{1.0e-8};
    double relative_tolerance{1.0e-8};
    int maximum_iterations{10000};
  };

  enum class Status {
    kConverged,
    kInvalidInput,
    kFactorizationFailure,
    kNonFinite,
    kMaximumIterations,
    kPrimalInfeasible,
    kDualInfeasible,
  };

  enum class StartMode { kCold, kWarm };
  enum class SetupMode { kCold, kWarm };

  struct Result {
    Status status{Status::kInvalidInput};
    int iterations{0};
    double primal_residual{0.0};
    double dual_residual{0.0};
    double stationarity_residual{0.0};
    Vector x{Vector::Zero()};

    [[nodiscard]] bool converged() const {
      return status == Status::kConverged;
    }
  };

  DenseQpSolver();
  explicit DenseQpSolver(Settings settings);
  ~DenseQpSolver();
  DenseQpSolver(DenseQpSolver&&) noexcept;
  DenseQpSolver& operator=(DenseQpSolver&&) noexcept;
  DenseQpSolver(const DenseQpSolver&) = delete;
  DenseQpSolver& operator=(const DenseQpSolver&) = delete;

  // Copies and validates min 0.5*x'H*x + g'x subject to l <= A*x <= u.
  // A warm update is retained only after a successful solve with an identical
  // equality-row mask.
  [[nodiscard]] Status setup(const Eigen::Ref<const Eigen::MatrixXd>& h,
                             const Eigen::Ref<const Eigen::VectorXd>& g,
                             const Eigen::Ref<const Eigen::MatrixXd>& a,
                             const Eigen::Ref<const Eigen::VectorXd>& lower,
                             const Eigen::Ref<const Eigen::VectorXd>& upper,
                             SetupMode setup_mode = SetupMode::kCold);

  // Cold starts are deterministic. A warm start is only used when requested.
  void reset();
  [[nodiscard]] Result solve(StartMode start_mode);
  [[nodiscard]] bool ready() const { return ready_; }

 private:
  class Impl;

  [[nodiscard]] bool validSettings() const;
  [[nodiscard]] bool isPositiveSemidefinite() const;
  [[nodiscard]] Result rejected(Status status) const;

  Settings settings_;
  int constraint_count_{0};
  int equality_count_{0};
  bool ready_{false};
  bool previous_solve_succeeded_{false};
  std::array<bool, kMaxConstraintCount> equality_mask_{};
  Matrix h_{Matrix::Zero()};
  Vector g_{Vector::Zero()};
  std::unique_ptr<Impl> impl_;
};

}  // namespace wheel_leg
