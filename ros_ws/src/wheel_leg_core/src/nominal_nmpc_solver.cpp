#include "wheel_leg_core/nominal_nmpc_solver.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/LU>

extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_ocp_phase23_nominal_nmpc_v2_4b1a6a0b.h"
}

namespace wheel_leg {
namespace {

constexpr int kHorizonSteps = 20;
constexpr int kStateSize = 12;
constexpr int kInputSize = 12;
constexpr int kParameterSize = 9;
constexpr double kStationarityTolerance = 1.0;
constexpr double kFeasibilityTolerance = 1.0e-3;
constexpr double kDefectTolerance = 1.0e-3;
constexpr double kProjectedStationarityTolerance = 0.05;
constexpr double kTerminalWeightMultiplier = 10.0;

using Clock = std::chrono::steady_clock;

constexpr std::array<double, kInputSize> kEquilibriumInput{
    -0.014600183648230658, -0.002144708393769802, 31.572223159792483,
    6.939287276081028, 0.3072127467744123, 0.00002385752452037761,
    0.014600183648233424, 0.002144708393775349, 31.549240840207528,
    -6.93345575287898, 0.39850336198734543, -0.000023857524519898827};
constexpr std::array<double, kInputSize> kInputLower{
    -15.0, -15.0, 10.0, 4.0, -2.0, -1.0,
    -15.0, -15.0, 10.0, -9.0, -2.0, -1.0};
constexpr std::array<double, kInputSize> kInputUpper{
    15.0, 15.0, 50.0, 9.0, 2.0, 1.0,
    15.0, 15.0, 50.0, -4.0, 2.0, 1.0};
constexpr std::array<double, kStateSize> kStateEnvelopeHalfWidth{
    0.02, 0.02, 0.01, 0.03, 0.03, 0.05,
    0.2, 0.2, 0.2, 0.5, 0.5, 0.5};
constexpr std::array<double, kStateSize> kStateCost{
    2500.0, 2500.0, 20000.0, 20000.0 / 9.0, 20000.0 / 9.0,
    200.0, 12.5, 12.5, 25.0, 1.0, 1.0, 1.0};
constexpr std::array<double, kInputSize> kInputCost{
    10.0, 10000.0, 1000000.0 / 225.0, 250000.0, 250000.0,
    1000000.0, 10.0, 10000.0, 1000000.0 / 225.0, 250000.0,
    250000.0, 1000000.0};
constexpr std::array<double, kInputSize> kInputScale{
    10.0, 10.0, 15.0, 2.0, 2.0, 1.0,
    10.0, 10.0, 15.0, 2.0, 2.0, 1.0};

bool finite(const NominalNmpcProblem &problem) {
  return problem.state.allFinite() && problem.reference.allFinite() &&
         problem.state_envelope_center.allFinite() &&
         problem.reference_rotation_n_from_b.allFinite() &&
         (problem.reference_rotation_n_from_b.transpose() *
              problem.reference_rotation_n_from_b -
          Eigen::Matrix3d::Identity())
                 .cwiseAbs()
                 .maxCoeff() <= 1.0e-9 &&
         std::abs(problem.reference_rotation_n_from_b.determinant() - 1.0) <=
             1.0e-9 &&
         problem.state.segment<3>(3).norm() <= 0.35;
}

double seconds(Clock::time_point start, Clock::time_point end) {
  return std::chrono::duration<double>(end - start).count();
}

}  // namespace

struct NominalNmpcSolver::Impl {
  using Capsule = ocp_phase23_nominal_nmpc_v2_4b1a6a0b_solver_capsule;

  Capsule *capsule{nullptr};
  ocp_nlp_config *config{nullptr};
  ocp_nlp_dims *dims{nullptr};
  ocp_nlp_in *input{nullptr};
  ocp_nlp_out *output{nullptr};
  ocp_nlp_solver *solver{nullptr};
  void *options{nullptr};
  bool cold{true};

  Impl() {
    capsule = ocp_phase23_nominal_nmpc_v2_4b1a6a0b_acados_create_capsule();
    if (capsule == nullptr ||
        ocp_phase23_nominal_nmpc_v2_4b1a6a0b_acados_create(capsule) != 0) {
      return;
    }
    config = ocp_phase23_nominal_nmpc_v2_4b1a6a0b_acados_get_nlp_config(capsule);
    dims = ocp_phase23_nominal_nmpc_v2_4b1a6a0b_acados_get_nlp_dims(capsule);
    input = ocp_phase23_nominal_nmpc_v2_4b1a6a0b_acados_get_nlp_in(capsule);
    output = ocp_phase23_nominal_nmpc_v2_4b1a6a0b_acados_get_nlp_out(capsule);
    solver = ocp_phase23_nominal_nmpc_v2_4b1a6a0b_acados_get_nlp_solver(capsule);
    options = ocp_phase23_nominal_nmpc_v2_4b1a6a0b_acados_get_nlp_opts(capsule);
  }

  ~Impl() {
    if (capsule != nullptr) {
      if (config != nullptr) {
        ocp_phase23_nominal_nmpc_v2_4b1a6a0b_acados_free(capsule);
      }
      ocp_phase23_nominal_nmpc_v2_4b1a6a0b_acados_free_capsule(capsule);
    }
  }

  [[nodiscard]] bool ready() const {
    return capsule != nullptr && config != nullptr && dims != nullptr &&
           input != nullptr && output != nullptr && solver != nullptr &&
           options != nullptr;
  }
};

NominalNmpcSolver::NominalNmpcSolver() : impl_(std::make_unique<Impl>()) {}
NominalNmpcSolver::~NominalNmpcSolver() = default;
NominalNmpcSolver::NominalNmpcSolver(NominalNmpcSolver &&) noexcept = default;
NominalNmpcSolver &NominalNmpcSolver::operator=(
    NominalNmpcSolver &&) noexcept = default;

bool NominalNmpcSolver::ready() const { return impl_ && impl_->ready(); }

void NominalNmpcSolver::reset() {
  if (!ready()) return;
  ocp_phase23_nominal_nmpc_v2_4b1a6a0b_acados_reset(
      impl_->capsule, 1, 1, 1, 1);
  impl_->cold = true;
}

NominalNmpcSolver::Result NominalNmpcSolver::solve(
    const NominalNmpcProblem &problem) {
  Result result;
  if (!ready()) return result;
  if (!finite(problem)) {
    result.status = Status::kInvalidInput;
    return result;
  }

  std::array<double, kParameterSize> parameters{};
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      parameters[3 * row + column] =
          problem.reference_rotation_n_from_b(row, column);
    }
  }
  for (int stage = 0; stage <= kHorizonSteps; ++stage) {
    if (ocp_phase23_nominal_nmpc_v2_4b1a6a0b_acados_update_params(
            impl_->capsule, stage, parameters.data(), kParameterSize) != 0) {
      result.status = Status::kPreparationFailed;
      return result;
    }
  }

  std::array<double, 24> yref{};
  std::copy(problem.reference.data(), problem.reference.data() + kStateSize,
            yref.begin());
  std::copy(kEquilibriumInput.begin(), kEquilibriumInput.end(),
            yref.begin() + kStateSize);
  for (int stage = 0; stage < kHorizonSteps; ++stage) {
    ocp_nlp_cost_model_set(
        impl_->config, impl_->dims, impl_->input, stage, "yref", yref.data());
  }
  ocp_nlp_cost_model_set(
      impl_->config, impl_->dims, impl_->input, kHorizonSteps, "yref",
      yref.data());
  ocp_nlp_constraints_model_set(
      impl_->config, impl_->dims, impl_->input, impl_->output, 0, "lbx",
      const_cast<double *>(problem.state.data()));
  ocp_nlp_constraints_model_set(
      impl_->config, impl_->dims, impl_->input, impl_->output, 0, "ubx",
      const_cast<double *>(problem.state.data()));
  std::array<double, kStateSize> envelope_lower{};
  std::array<double, kStateSize> envelope_upper{};
  for (int index = 0; index < kStateSize; ++index) {
    envelope_lower[index] = problem.state_envelope_center[index] -
                            kStateEnvelopeHalfWidth[index];
    envelope_upper[index] = problem.state_envelope_center[index] +
                            kStateEnvelopeHalfWidth[index];
  }
  for (int stage = 1; stage <= kHorizonSteps; ++stage) {
    ocp_nlp_constraints_model_set(impl_->config, impl_->dims, impl_->input,
                                  impl_->output, stage, "lbx",
                                  envelope_lower.data());
    ocp_nlp_constraints_model_set(impl_->config, impl_->dims, impl_->input,
                                  impl_->output, stage, "ubx",
                                  envelope_upper.data());
  }

  if (impl_->cold) {
    for (int stage = 0; stage < kHorizonSteps; ++stage) {
      ocp_nlp_out_set(
          impl_->config, impl_->dims, impl_->output, impl_->input, stage, "x",
          const_cast<double *>(problem.state.data()));
      ocp_nlp_out_set(
          impl_->config, impl_->dims, impl_->output, impl_->input, stage, "u",
          const_cast<double *>(kEquilibriumInput.data()));
    }
    ocp_nlp_out_set(
        impl_->config, impl_->dims, impl_->output, impl_->input, kHorizonSteps,
        "x", const_cast<double *>(problem.state.data()));
  }

  int phase = 0;
  ocp_nlp_solver_opts_set(impl_->config, impl_->options, "rti_phase", &phase);
  const auto solve_start = Clock::now();
  result.acados_status =
      ocp_phase23_nominal_nmpc_v2_4b1a6a0b_acados_solve(impl_->capsule);
  const auto solve_end = Clock::now();
  if (result.acados_status != 0) {
    result.status = Status::kFeedbackFailed;
    return result;
  }
  ocp_nlp_get(impl_->solver, "time_lin", &result.preparation_time_s);
  result.feedback_time_s = std::max(
      0.0, seconds(solve_start, solve_end) - result.preparation_time_s);

  ocp_nlp_out_get(
      impl_->config, impl_->dims, impl_->output, 0, "u",
      result.wrench_flu.data());
  std::array<NominalNmpcModel::State, kHorizonSteps + 1> states;
  std::array<NominalNmpcModel::Input, kHorizonSteps> inputs;
  for (int stage = 0; stage <= kHorizonSteps; ++stage) {
    ocp_nlp_out_get(impl_->config, impl_->dims, impl_->output, stage, "x",
                    states[stage].data());
    if (stage < kHorizonSteps) {
      ocp_nlp_out_get(impl_->config, impl_->dims, impl_->output, stage, "u",
                      inputs[stage].data());
    }
  }
  ocp_nlp_eval_residuals(impl_->solver, impl_->input, impl_->output);
  ocp_nlp_get(impl_->solver, "res_stat", &result.stationarity_residual);
  ocp_nlp_get(impl_->solver, "res_eq", &result.dynamics_residual);
  ocp_nlp_get(impl_->solver, "res_ineq", &result.inequality_residual);
  ocp_nlp_get(impl_->solver, "res_comp", &result.complementarity_residual);

  std::array<NominalNmpcModel::Result, kHorizonSteps> models;
  double bound_violation = 0.0;
  result.maximum_dynamics_defect =
      (states.front() - problem.state).cwiseAbs().maxCoeff();
  for (int stage = 0; stage < kHorizonSteps; ++stage) {
    models[stage] = NominalNmpcModel{}.evaluate(
        states[stage], inputs[stage], problem.reference_rotation_n_from_b);
    if (!models[stage].ok()) {
      result.maximum_dynamics_defect =
          std::numeric_limits<double>::infinity();
      break;
    }
    const double defect =
        (models[stage].next - states[stage + 1]).cwiseAbs().maxCoeff();
    result.maximum_dynamics_defect =
        std::max(result.maximum_dynamics_defect, defect);
    for (int index = 0; index < kInputSize; ++index) {
      bound_violation = std::max(
          bound_violation,
          std::max(kInputLower[index] - inputs[stage][index],
                   inputs[stage][index] - kInputUpper[index]));
      const double error = inputs[stage][index] - kEquilibriumInput[index];
      result.objective += 0.5 * kInputCost[index] * error * error;
    }
    for (int index = 0; index < kStateSize; ++index) {
      result.state_bound_violation = std::max(
          result.state_bound_violation,
          std::max(envelope_lower[index] - states[stage + 1][index],
                   states[stage + 1][index] - envelope_upper[index]));
      const double error = states[stage][index] - problem.reference[index];
      result.objective += 0.5 * kStateCost[index] * error * error;
    }
  }
  result.first_step_defect = models.front().ok()
      ? (models.front().next - states[1]).cwiseAbs().maxCoeff()
      : std::numeric_limits<double>::infinity();
  NominalNmpcModel::State costate;
  for (int index = 0; index < kStateSize; ++index) {
    const double error = states.back()[index] - problem.reference[index];
    costate[index] = kTerminalWeightMultiplier * kStateCost[index] * error;
    result.objective += 0.5 * kTerminalWeightMultiplier *
                        kStateCost[index] * error * error;
  }
  for (int stage = kHorizonSteps - 1; stage >= 0; --stage) {
    const NominalNmpcModel::Input gradient =
        (inputs[stage] - Eigen::Map<const NominalNmpcModel::Input>(
             kEquilibriumInput.data()))
            .cwiseProduct(Eigen::Map<const NominalNmpcModel::Input>(
                kInputCost.data())) +
        models[stage].discrete_input_jacobian.transpose() * costate;
    for (int index = 0; index < kInputSize; ++index) {
      const double projected = std::clamp(
          inputs[stage][index] - gradient[index] / kInputCost[index],
          kInputLower[index], kInputUpper[index]);
      result.projected_stationarity_residual = std::max(
          result.projected_stationarity_residual,
          std::abs(inputs[stage][index] - projected) / kInputScale[index]);
    }
    NominalNmpcModel::State state_gradient;
    for (int index = 0; index < kStateSize; ++index) {
      state_gradient[index] =
          kStateCost[index] * (states[stage][index] - problem.reference[index]);
    }
    costate = state_gradient +
              models[stage].discrete_state_jacobian.transpose() * costate;
  }
  result.input_bound_violation = bound_violation;
  const std::array<double, 12> audit_values{
      result.stationarity_residual, result.dynamics_residual,
      result.inequality_residual, result.complementarity_residual,
      result.first_step_defect, result.maximum_dynamics_defect,
      result.input_bound_violation, result.state_bound_violation,
      result.objective,
      result.projected_stationarity_residual, result.preparation_time_s,
      result.feedback_time_s};
  if (!result.wrench_flu.allFinite() ||
      !std::all_of(audit_values.begin(), audit_values.end(),
                   [](double value) { return std::isfinite(value); }) ||
      result.stationarity_residual > kStationarityTolerance ||
      std::max({result.dynamics_residual, result.inequality_residual,
                result.complementarity_residual}) >
          kFeasibilityTolerance ||
      result.maximum_dynamics_defect > kDefectTolerance ||
      result.projected_stationarity_residual >
          kProjectedStationarityTolerance ||
      bound_violation > 1.0e-8 || result.state_bound_violation > 1.0e-8) {
    result.status = Status::kAuditFailed;
    return result;
  }
  impl_->cold = false;
  result.status = Status::kOk;
  return result;
}

}  // namespace wheel_leg
