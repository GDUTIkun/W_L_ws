#pragma once

#include <memory>

#include <Eigen/Core>

#include "wheel_leg_core/wheel_aware_nmpc_model.hpp"

namespace wheel_leg {

struct WheelAwareNmpcProblem {
  WheelAwareNmpcModel::State state{WheelAwareNmpcModel::State::Zero()};
  WheelAwareNmpcModel::State reference{WheelAwareNmpcModel::State::Zero()};
  WheelAwareNmpcModel::State state_envelope_center{WheelAwareNmpcModel::State::Zero()};
  Eigen::Matrix3d reference_rotation_n_from_b{Eigen::Matrix3d::Identity()};
};

class WheelAwareNmpcSolver {
 public:
  enum class Status { kOk, kNotReady, kInvalidInput, kSolveFailed, kAuditFailed };

  struct Result {
    Status status{Status::kNotReady};
    int acados_status{-1};
    WheelAwareNmpcModel::Input interaction_wrench_flu{WheelAwareNmpcModel::Input::Zero()};
    double solve_time_s{0.0};
    double stationarity_residual{0.0};
    double dynamics_residual{0.0};
    double inequality_residual{0.0};
    double complementarity_residual{0.0};
    double first_step_defect{0.0};
    double maximum_dynamics_defect{0.0};
    double input_bound_violation{0.0};
    double state_bound_violation{0.0};
    double projected_stationarity_residual{0.0};
    double objective{0.0};
    [[nodiscard]] bool ok() const { return status == Status::kOk; }
  };

  WheelAwareNmpcSolver();
  ~WheelAwareNmpcSolver();
  WheelAwareNmpcSolver(const WheelAwareNmpcSolver &) = delete;
  WheelAwareNmpcSolver &operator=(const WheelAwareNmpcSolver &) = delete;
  WheelAwareNmpcSolver(WheelAwareNmpcSolver &&) noexcept;
  WheelAwareNmpcSolver &operator=(WheelAwareNmpcSolver &&) noexcept;

  [[nodiscard]] bool ready() const;
  void reset();
  [[nodiscard]] Result solve(const WheelAwareNmpcProblem &problem);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace wheel_leg
