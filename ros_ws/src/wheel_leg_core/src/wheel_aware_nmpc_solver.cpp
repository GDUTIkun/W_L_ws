#include "wheel_leg_core/wheel_aware_nmpc_solver.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>

#include <Eigen/LU>

extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_ocp_phase27_wheel_aware_nmpc_v2_ece6e123.h"
}

namespace wheel_leg {
namespace {

constexpr int kN = 20;
constexpr int kNx = 16;
constexpr int kNu = 12;
constexpr int kNp = 9;
constexpr double kStepS = 0.02;
constexpr double kTerminalMultiplier = 10.0;
constexpr double kStationarityTolerance = 1.0;
constexpr double kFeasibilityTolerance = 1.0e-3;
constexpr double kDefectTolerance = 1.0e-3;
constexpr double kProjectedStationarityTolerance = 0.05;
using Clock = std::chrono::steady_clock;

constexpr std::array<double, kNu> kEquilibriumInput{
    0.0, 0.0, 27.675229491866027, 0.11327183296816838, 0.0, 0.0,
    0.0, 0.0, 28.714612508133982, 0.11327183296816838, 0.0, 0.0};
constexpr std::array<double, kNu> kInputLower{
    -15.0, -15.0, 10.0, -4.0, -2.0, -1.0,
    -15.0, -15.0, 10.0, -4.0, -2.0, -1.0};
constexpr std::array<double, kNu> kInputUpper{
    15.0, 15.0, 50.0, 4.0, 2.0, 1.0,
    15.0, 15.0, 50.0, 4.0, 2.0, 1.0};
constexpr std::array<double, kNx> kEnvelope{
    0.12, 0.08, 0.03, 0.08, 0.08, 0.10, 0.4, 0.4,
    0.4, 0.8, 0.8, 0.8, 0.08, 0.08, 0.3, 0.3};
constexpr std::array<double, 2> kWheelLower{-0.3303432354, -0.3321211483};
constexpr std::array<double, 2> kWheelUpper{0.1678677251, 0.1659029424};
constexpr std::array<double, kNx> kStateCost{
    625.0, 625.0, 20000.0, 20000.0 / 9.0, 20000.0 / 9.0,
    200.0, 12.5, 12.5, 25.0, 1.0, 1.0, 1.0,
    5000.0, 5000.0, 400.0 / 9.0, 400.0 / 9.0};
constexpr std::array<double, kNu> kInputCost{
    10.0, 10000.0, 1000000.0 / 225.0, 250000.0, 250000.0, 1000000.0,
    10.0, 10000.0, 1000000.0 / 225.0, 250000.0, 250000.0, 1000000.0};
constexpr std::array<double, kNu> kInputScale{
    10.0, 10.0, 15.0, 2.0, 2.0, 1.0,
    10.0, 10.0, 15.0, 2.0, 2.0, 1.0};

WheelAwareNmpcModel::State advanceReference(
    const WheelAwareNmpcModel::State &initial, int stage) {
  auto result = initial;
  const double time = kStepS * stage;
  result.segment<3>(0) += time * initial.segment<3>(6);
  result.segment<3>(3) += time * initial.segment<3>(9);
  result[12] += time * initial[14];
  result[13] += time * initial[15];
  return result;
}

bool finite(const WheelAwareNmpcProblem &problem) {
  if (!problem.state.allFinite() || !problem.reference.allFinite() ||
      !problem.state_envelope_center.allFinite() ||
      !problem.reference_rotation_n_from_b.allFinite()) return false;
  const auto &rotation = problem.reference_rotation_n_from_b;
  if ((rotation.transpose() * rotation - Eigen::Matrix3d::Identity())
          .cwiseAbs().maxCoeff() > 1.0e-9 ||
      std::abs(rotation.determinant() - 1.0) > 1.0e-9) return false;
  for (const auto *state : {&problem.state, &problem.reference,
                            &problem.state_envelope_center}) {
    if (state->segment<3>(3).norm() > 0.35 ||
        (*state)[12] < kWheelLower[0] || (*state)[12] > kWheelUpper[0] ||
        (*state)[13] < kWheelLower[1] || (*state)[13] > kWheelUpper[1]) return false;
  }
  for (int stage = 0; stage <= kN; ++stage) {
    const auto reference = advanceReference(problem.reference, stage);
    if (reference.segment<3>(3).norm() > 0.35 ||
        reference[12] < kWheelLower[0] || reference[12] > kWheelUpper[0] ||
        reference[13] < kWheelLower[1] || reference[13] > kWheelUpper[1]) return false;
  }
  return true;
}

}  // namespace

struct WheelAwareNmpcSolver::Impl {
  using Capsule = ocp_phase27_wheel_aware_nmpc_v2_ece6e123_solver_capsule;
  Capsule *capsule{nullptr};
  ocp_nlp_config *config{nullptr};
  ocp_nlp_dims *dims{nullptr};
  ocp_nlp_in *input{nullptr};
  ocp_nlp_out *output{nullptr};
  ocp_nlp_solver *solver{nullptr};
  bool cold{true};
  Impl() {
    capsule = ocp_phase27_wheel_aware_nmpc_v2_ece6e123_acados_create_capsule();
    if (capsule == nullptr ||
        ocp_phase27_wheel_aware_nmpc_v2_ece6e123_acados_create(capsule) != 0) return;
    config = ocp_phase27_wheel_aware_nmpc_v2_ece6e123_acados_get_nlp_config(capsule);
    dims = ocp_phase27_wheel_aware_nmpc_v2_ece6e123_acados_get_nlp_dims(capsule);
    input = ocp_phase27_wheel_aware_nmpc_v2_ece6e123_acados_get_nlp_in(capsule);
    output = ocp_phase27_wheel_aware_nmpc_v2_ece6e123_acados_get_nlp_out(capsule);
    solver = ocp_phase27_wheel_aware_nmpc_v2_ece6e123_acados_get_nlp_solver(capsule);
  }
  ~Impl() {
    if (capsule == nullptr) return;
    if (config != nullptr) ocp_phase27_wheel_aware_nmpc_v2_ece6e123_acados_free(capsule);
    ocp_phase27_wheel_aware_nmpc_v2_ece6e123_acados_free_capsule(capsule);
  }
  [[nodiscard]] bool ready() const {
    return capsule && config && dims && input && output && solver;
  }
};

WheelAwareNmpcSolver::WheelAwareNmpcSolver() : impl_(std::make_unique<Impl>()) {}
WheelAwareNmpcSolver::~WheelAwareNmpcSolver() = default;
WheelAwareNmpcSolver::WheelAwareNmpcSolver(WheelAwareNmpcSolver &&) noexcept = default;
WheelAwareNmpcSolver &WheelAwareNmpcSolver::operator=(WheelAwareNmpcSolver &&) noexcept = default;
bool WheelAwareNmpcSolver::ready() const { return impl_ && impl_->ready(); }
void WheelAwareNmpcSolver::reset() {
  if (!ready()) return;
  ocp_phase27_wheel_aware_nmpc_v2_ece6e123_acados_reset(impl_->capsule, 1, 1, 1, 1);
  impl_->cold = true;
}

WheelAwareNmpcSolver::Result WheelAwareNmpcSolver::solve(
    const WheelAwareNmpcProblem &problem) {
  Result result;
  if (!ready()) return result;
  if (!finite(problem)) { result.status = Status::kInvalidInput; return result; }

  std::array<double, kNp> parameters{};
  for (int row = 0; row < 3; ++row)
    for (int column = 0; column < 3; ++column)
      parameters[3 * row + column] = problem.reference_rotation_n_from_b(row, column);
  for (int stage = 0; stage <= kN; ++stage)
    if (ocp_phase27_wheel_aware_nmpc_v2_ece6e123_acados_update_params(
            impl_->capsule, stage, parameters.data(), kNp) != 0) {
      result.status = Status::kSolveFailed; return result;
    }

  std::array<WheelAwareNmpcModel::State, kN + 1> references;
  std::array<WheelAwareNmpcModel::State, kN + 1> lower;
  std::array<WheelAwareNmpcModel::State, kN + 1> upper;
  std::array<double, kNx + kNu> yref{};
  for (int stage = 0; stage <= kN; ++stage) {
    references[stage] = advanceReference(problem.reference, stage);
    const auto center = advanceReference(problem.state_envelope_center, stage);
    for (int index = 0; index < kNx; ++index) {
      lower[stage][index] = center[index] - kEnvelope[index];
      upper[stage][index] = center[index] + kEnvelope[index];
    }
    lower[stage][12] = std::max(lower[stage][12], kWheelLower[0]);
    lower[stage][13] = std::max(lower[stage][13], kWheelLower[1]);
    upper[stage][12] = std::min(upper[stage][12], kWheelUpper[0]);
    upper[stage][13] = std::min(upper[stage][13], kWheelUpper[1]);
    std::copy(references[stage].data(), references[stage].data() + kNx, yref.begin());
    std::copy(kEquilibriumInput.begin(), kEquilibriumInput.end(), yref.begin() + kNx);
    ocp_nlp_cost_model_set(impl_->config, impl_->dims, impl_->input, stage, "yref", yref.data());
    if (stage > 0) {
      ocp_nlp_constraints_model_set(impl_->config, impl_->dims, impl_->input,
                                    impl_->output, stage, "lbx", lower[stage].data());
      ocp_nlp_constraints_model_set(impl_->config, impl_->dims, impl_->input,
                                    impl_->output, stage, "ubx", upper[stage].data());
    }
  }
  ocp_nlp_constraints_model_set(impl_->config, impl_->dims, impl_->input,
                                impl_->output, 0, "lbx", const_cast<double *>(problem.state.data()));
  ocp_nlp_constraints_model_set(impl_->config, impl_->dims, impl_->input,
                                impl_->output, 0, "ubx", const_cast<double *>(problem.state.data()));

  if (impl_->cold) {
    for (int stage = 0; stage < kN; ++stage) {
      ocp_nlp_out_set(impl_->config, impl_->dims, impl_->output, impl_->input,
                      stage, "x", references[stage].data());
      ocp_nlp_out_set(impl_->config, impl_->dims, impl_->output, impl_->input,
                      stage, "u", const_cast<double *>(kEquilibriumInput.data()));
    }
    ocp_nlp_out_set(impl_->config, impl_->dims, impl_->output, impl_->input,
                    kN, "x", references[kN].data());
  }
  const auto start = Clock::now();
  result.acados_status =
      ocp_phase27_wheel_aware_nmpc_v2_ece6e123_acados_solve(impl_->capsule);
  result.solve_time_s = std::chrono::duration<double>(Clock::now() - start).count();
  if (result.acados_status != 0) { result.status = Status::kSolveFailed; return result; }

  ocp_nlp_out_get(impl_->config, impl_->dims, impl_->output, 0, "u",
                  result.interaction_wrench_flu.data());
  std::array<WheelAwareNmpcModel::State, kN + 1> states;
  std::array<WheelAwareNmpcModel::Input, kN> inputs;
  std::array<WheelAwareNmpcModel::Result, kN> models;
  for (int stage = 0; stage <= kN; ++stage) {
    ocp_nlp_out_get(impl_->config, impl_->dims, impl_->output, stage, "x", states[stage].data());
    if (stage < kN)
      ocp_nlp_out_get(impl_->config, impl_->dims, impl_->output, stage, "u", inputs[stage].data());
  }
  ocp_nlp_eval_residuals(impl_->solver, impl_->input, impl_->output);
  ocp_nlp_get(impl_->solver, "res_stat", &result.stationarity_residual);
  ocp_nlp_get(impl_->solver, "res_eq", &result.dynamics_residual);
  ocp_nlp_get(impl_->solver, "res_ineq", &result.inequality_residual);
  ocp_nlp_get(impl_->solver, "res_comp", &result.complementarity_residual);
  result.maximum_dynamics_defect = (states[0] - problem.state).cwiseAbs().maxCoeff();
  for (int stage = 0; stage < kN; ++stage) {
    models[stage] = WheelAwareNmpcModel{}.evaluate(states[stage], inputs[stage], problem.reference_rotation_n_from_b);
    if (!models[stage].ok()) { result.maximum_dynamics_defect = std::numeric_limits<double>::infinity(); break; }
    const double defect = (models[stage].next - states[stage + 1]).cwiseAbs().maxCoeff();
    result.maximum_dynamics_defect = std::max(result.maximum_dynamics_defect, defect);
    for (int index = 0; index < kNu; ++index) {
      result.input_bound_violation = std::max(result.input_bound_violation,
          std::max(kInputLower[index] - inputs[stage][index], inputs[stage][index] - kInputUpper[index]));
      const double error = inputs[stage][index] - kEquilibriumInput[index];
      result.objective += 0.5 * kStepS * kInputCost[index] * error * error;
    }
    for (int index = 0; index < kNx; ++index) {
      result.state_bound_violation = std::max(result.state_bound_violation,
          std::max(lower[stage + 1][index] - states[stage + 1][index],
                   states[stage + 1][index] - upper[stage + 1][index]));
      const double error = states[stage][index] - references[stage][index];
      result.objective += 0.5 * kStepS * kStateCost[index] * error * error;
    }
  }
  result.first_step_defect = models[0].ok()
      ? (models[0].next - states[1]).cwiseAbs().maxCoeff()
      : std::numeric_limits<double>::infinity();
  WheelAwareNmpcModel::State costate;
  for (int index = 0; index < kNx; ++index) {
    const double error = states[kN][index] - references[kN][index];
    costate[index] = kTerminalMultiplier * kStateCost[index] * error;
    result.objective += 0.5 * kTerminalMultiplier * kStateCost[index] * error * error;
  }
  for (int stage = kN - 1; stage >= 0; --stage) {
    WheelAwareNmpcModel::Input gradient;
    for (int index = 0; index < kNu; ++index)
      gradient[index] = kStepS * kInputCost[index] *
                        (inputs[stage][index] - kEquilibriumInput[index]);
    gradient += models[stage].discrete_input_jacobian.transpose() * costate;
    for (int index = 0; index < kNu; ++index) {
      const double projected = std::clamp(inputs[stage][index] -
                                              gradient[index] /
                                                  (kStepS * kInputCost[index]),
                                          kInputLower[index], kInputUpper[index]);
      result.projected_stationarity_residual = std::max(
          result.projected_stationarity_residual,
          std::abs(inputs[stage][index] - projected) / kInputScale[index]);
    }
    WheelAwareNmpcModel::State state_gradient;
    for (int index = 0; index < kNx; ++index)
      state_gradient[index] = kStepS * kStateCost[index] *
                              (states[stage][index] - references[stage][index]);
    costate = state_gradient + models[stage].discrete_state_jacobian.transpose() * costate;
  }
  const std::array<double, 11> audit{result.solve_time_s, result.stationarity_residual,
      result.dynamics_residual, result.inequality_residual, result.complementarity_residual,
      result.first_step_defect, result.maximum_dynamics_defect, result.input_bound_violation,
      result.state_bound_violation, result.projected_stationarity_residual, result.objective};
  if (!result.interaction_wrench_flu.allFinite() ||
      !std::all_of(audit.begin(), audit.end(), [](double value) { return std::isfinite(value); }) ||
      result.stationarity_residual > kStationarityTolerance ||
      std::max({result.dynamics_residual, result.inequality_residual, result.complementarity_residual}) > kFeasibilityTolerance ||
      result.maximum_dynamics_defect > kDefectTolerance ||
      result.projected_stationarity_residual > kProjectedStationarityTolerance ||
      result.input_bound_violation > 1.0e-8 || result.state_bound_violation > 1.0e-8) {
    result.status = Status::kAuditFailed; return result;
  }
  impl_->cold = false;
  result.status = Status::kOk;
  return result;
}

}  // namespace wheel_leg
