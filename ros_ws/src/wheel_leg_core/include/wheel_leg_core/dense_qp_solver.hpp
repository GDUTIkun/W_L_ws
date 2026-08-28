#pragma once

#include <Eigen/Core>
#include <Eigen/Cholesky>

namespace wheel_leg {

// Fixed-capacity dense ADMM solver for the Phase 21 bound-form QP.
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
    double rho{1.0};
    double sigma{1.0e-6};
    double relaxation{1.6};
    double absolute_tolerance{1.0e-7};
    double relative_tolerance{1.0e-7};
    int maximum_iterations{400};
  };

  enum class Status {
    kConverged,
    kInvalidInput,
    kFactorizationFailure,
    kNonFinite,
    kMaximumIterations,
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

  // Copies and validates min 0.5*x'H*x + g'x subject to l <= A*x <= u.
  // kWarm retains x/z/y only when the previous row count matches.
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
  [[nodiscard]] bool validSettings() const;
  [[nodiscard]] bool isPositiveSemidefinite() const;
  [[nodiscard]] Result rejected(Status status) const;

  Settings settings_;
  int constraint_count_{0};
  bool ready_{false};
  Matrix h_{Matrix::Zero()};
  Vector g_{Vector::Zero()};
  ConstraintMatrix a_{ConstraintMatrix::Zero()};
  ConstraintVector lower_{ConstraintVector::Zero()};
  ConstraintVector upper_{ConstraintVector::Zero()};
  Matrix system_{Matrix::Zero()};
  Eigen::LDLT<Matrix> factorization_;

  Vector x_{Vector::Zero()};
  Vector rhs_{Vector::Zero()};
  ConstraintVector z_{ConstraintVector::Zero()};
  ConstraintVector z_previous_{ConstraintVector::Zero()};
  ConstraintVector y_{ConstraintVector::Zero()};
  ConstraintVector ax_{ConstraintVector::Zero()};
  ConstraintVector residual_{ConstraintVector::Zero()};
  ConstraintVector relaxed_{ConstraintVector::Zero()};
  Vector dual_work_{Vector::Zero()};
};

}  // namespace wheel_leg
