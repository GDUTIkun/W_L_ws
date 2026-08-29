#pragma once

#include <memory>

#include <Eigen/Core>

#include "wheel_leg_core/nominal_nmpc_model.hpp"

namespace wheel_leg {

struct NominalNmpcProblem {
  NominalNmpcModel::State state{NominalNmpcModel::State::Zero()};
  NominalNmpcModel::State reference{NominalNmpcModel::State::Zero()};
  NominalNmpcModel::State state_envelope_center{
      NominalNmpcModel::State::Zero()};
  Eigen::Matrix3d reference_rotation_n_from_b{Eigen::Matrix3d::Identity()};
};

class NominalNmpcSolver {
 public:
  enum class Status {
    kOk,
    kNotReady,
    kInvalidInput,
    kPreparationFailed,
    kFeedbackFailed,
    kAuditFailed,
  };

  struct Result {
    Status status{Status::kNotReady};
    int acados_status{-1};
    NominalNmpcModel::Input wrench_flu{NominalNmpcModel::Input::Zero()};
    double preparation_time_s{0.0};
    double feedback_time_s{0.0};
    double stationarity_residual{0.0};
    double dynamics_residual{0.0};
    double inequality_residual{0.0};
    double complementarity_residual{0.0};
    double first_step_defect{0.0};
    double maximum_dynamics_defect{0.0};
    double input_bound_violation{0.0};
    double state_bound_violation{0.0};
    double objective{0.0};
    double projected_stationarity_residual{0.0};

    [[nodiscard]] bool ok() const { return status == Status::kOk; }
  };

  NominalNmpcSolver();
  ~NominalNmpcSolver();
  NominalNmpcSolver(const NominalNmpcSolver &) = delete;
  NominalNmpcSolver &operator=(const NominalNmpcSolver &) = delete;
  NominalNmpcSolver(NominalNmpcSolver &&) noexcept;
  NominalNmpcSolver &operator=(NominalNmpcSolver &&) noexcept;

  [[nodiscard]] bool ready() const;
  void reset();
  [[nodiscard]] Result solve(const NominalNmpcProblem &problem);

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace wheel_leg
