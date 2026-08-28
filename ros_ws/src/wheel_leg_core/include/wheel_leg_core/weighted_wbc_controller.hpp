#pragma once

#include "wheel_leg_core/dense_qp_solver.hpp"
#include "wheel_leg_core/weighted_wbc_problem.hpp"

namespace wheel_leg {

class WeightedWbcController {
 public:
  enum class Task : std::size_t {
    kContact, kBaseX, kHeight, kOrientation, kLeg, kWrenchFidelity,
    kSlackPenalty, kCount,
  };
  static constexpr std::size_t kTaskCount = static_cast<std::size_t>(Task::kCount);
  enum class Status {
    kOk,
    kModelRejected,
    kProblemRejected,
    kSolverRejected,
    kHardViolation,
    kNonFinite,
  };

  struct Result {
    Status status{Status::kProblemRejected};
    NominalWbcModel::Status model_status{NominalWbcModel::Status::kInvalidState};
    DenseQpSolver::Status solver_status{DenseQpSolver::Status::kInvalidInput};
    JointVector torque_nm{};
    int iterations{0};
    double hard_violation{0.0};
    double stationarity_residual{0.0};
    double primal_residual{0.0};
    double dual_residual{0.0};
    NominalWbcModel::Diagnostics model_diagnostics{};
    Eigen::Matrix<double, 42, 1> physical_solution{
        Eigen::Matrix<double, 42, 1>::Zero()};
    std::array<double, kTaskCount> task_max_abs_normalized_residual{};
    std::array<double, kTaskCount> task_normalized_squared_cost{};
    double maximum_normalized_slack{0.0};

    [[nodiscard]] bool ok() const { return status == Status::kOk; }
  };

  WeightedWbcController();
  void reset();
  [[nodiscard]] Result step(const RobotState &state,
                            const WbcReference &reference);

 private:
  NominalWbcModel model_{};
  WeightedWbcProblem problem_{};
  DenseQpSolver solver_;
  bool warm_start_available_{false};
};

}  // namespace wheel_leg
