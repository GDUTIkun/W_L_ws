# Phase 34: 12D Base NMPC + WBC Wheel Tracking Feasibility — PLAN

Status: `review`

Design input: user-provided architecture proposal, accepted only as a feasibility hypothesis. Live
source and frozen Phase 27–33 evidence override conflicting statements below. This PLAN does not
approve implementation results, retuning, production selection, or any fallback architecture.

## Goal

Determine whether an opt-in `12D base NMPC + existing common wheel-position planner + full-body
WBC longitudinal wheel tracking` candidate can produce restorative Phase29 T0/T1 actions and bounded
current-nominal MuJoCo closed-loop behavior while preserving the Phase27 internal-wrench contract.

## Design Audit

The proposed responsibility split is technically testable, but it is not present in the current
runtime and cannot be obtained by truncating an existing artifact:

- **Architecture boundary — hypothesis accepted:** removing `xi_L/xi_R/dxi_L/dxi_R` and Eq.(12)
  from the OCP avoids requiring the Phase32-failed 16-state wheel Markov closure inside NMPC. It does
  not prove that the lower-level plant is closed or that WBC can track wheel position while realizing
  the requested wrench; those are Phase34 evidence gates.
- **12D model — current Phase23 artifact is not reusable:** `NominalNmpcModel` uses a locked-composite
  `6.4344 kg` body and interprets each input moment directly about the base-control point. Phase27
  instead uses the wheel-excluded body (`5.7482 kg`) and wheel-on-body internal wrenches applied at
  the two wheel-body origins. Phase34 therefore needs an append-only 12D projection of the Phase27
  base equations, not the Phase23 model/artifact and not the design-input `7 kg` value.
- **Geometry — two measured parameters are required:** Phase27 angular dynamics contains
  `r_i(xi_i) cross F_i`; deleting the wheel states while dropping `xi_L/xi_R` would change the base
  moment map. The first candidate supplies current measured `xi_L/xi_R` as read-only per-solve OCP
  parameters, held ZOH across the horizon. Failure of that bounded approximation is a Phase34 FAIL,
  not permission to augment state or restore Eq.(12).
- **Planner — common channel only:** live `WheelPositionPlanner` outputs only common position,
  velocity and acceleration with `0.15 m/s`, `0.5 m/s^2` and intersection workspace clamps. It has no
  differential planner/reference. Phase34 preserves that planner; the WBC differential channel may
  only hold the reset-time measured differential coordinate with zero velocity/acceleration target.
- **Runtime dataflow — proposed bypass does not exist:** current ControllerCore writes planner output
  into the 16D NMPC reference and sends only the solver's internal-wrench request to Phase27 Minimal
  WBC. No planner value currently reaches `WbcReference`.
- **WBC task — absent:** current `NominalWbcModel` exposes longitudinal wheel position/velocity but
  only the Phase33 diagnostic vertical coordinate has an affine acceleration map. Phase27 Minimal WBC
  has soft contact-acceleration and interaction-wrench/slack objectives; it has no longitudinal wheel
  tracking row. Phase34 must derive and independently validate `ddxi=A_xi*nudot+b_xi` before adding an
  isolated opt-in soft task.
- **OCP cost — design values conflict with production:** the live Phase27 v2 state diagonal is
  `[625,625,20000,20000/9,20000/9,200,12.5,12.5,25,1,1,1,5000,5000,
  400/9,400/9]`, not the design-input `[25,10,80,...]`. The candidate keeps the actual first twelve
  entries, the actual input cost/bounds and terminal multiplier `10`. The live OCP has no `Delta u`
  cost; Phase34 must not introduce one under the label “unchanged”.
- **Solver/discretization — preserve Phase27:** `20 ms`, `N=20`, `0.4 s`, two fixed `10 ms` RK4
  substeps, SQP-RTI and partial-condensing HPIPM remain fixed. The 12D state envelope is exactly the
  first twelve Phase27 rows.
- **Workspace protection — intentionally weaker candidate:** removing wheel states also removes
  predicted wheel-workspace bounds from the OCP. Current-state WBC/model rejection and planner clamps
  remain, but Phase34 cannot claim production safety or predictive workspace protection. No
  workspace feedback or NMPC parameter feedback may be added in this Phase.
- **Motion range — design cases corrected:** `ControllerCore::setPhase27MotionReference` accepts only
  `|v|<=0.20 m/s` and `|yaw rate|<=0.08 rad/s`. The proposed `0.5/1.0 m/s` cases are outside the live
  contract and are replaced by bounded `0.1/0.2 m/s` cases; limits are not widened.
- **Prior evidence — not silently reused as approval:** Phase29 froze T0 as terminal
  base-longitudinal propagation (`P29-E`) and T1 as attitude-dominant, wheel-rate-secondary coupling
  (`P29-D`). Removing four wheel states may leave T0 or T1 non-restorative. Phase33's vertical
  cross-side gate failure neither proves nor disproves longitudinal authority, but requires an
  authority-before-gains order and full 2x2 common/differential reporting.

Grounding used CBM project `W_L_ws`, generation `2026-08-29T06:47:42Z`, Verify tier. All cited live
paths have no recorded coverage gap; the Phase33-modified WBC files report changed metadata and were
therefore read directly. `docs/`, `tools/` and configs were read directly because they are outside the
main code index. Existing Graphify history confirms the Phase23/27 model and wrench-contract split;
live source remains authoritative.

## Current State

- Phase27 production remains a 16-state wheel-aware acados NMPC feeding a 42-variable/104-hard-row
  Phase27 Minimal ProxQP WBC on the `2/10/20 ms` physics/WBC/NMPC schedule.
- Phase29 provides authoritative production-lifecycle T0/T1 snapshots, exact corrective-direction
  definitions and an offline converged-SQP comparison path.
- Phase32 proved the current floating-base mesh-contact plant is not Markov-closed in x16; x24 is
  only a necessary observable superset, not an approved candidate.
- Phase33 is REVIEW/REWORK. Its vertical diagnostic profile is not production and must not be folded
  into Phase34; its uncommitted worktree changes must be preserved.
- Missing are the Phase27-semantics 12D parameterized model/OCP, longitudinal affine WBC map/task,
  planner-to-WBC reference path and any evidence that their composition is viable.

## Scope

- Freeze append-only 12D base-state/model/OCP contracts with current `xi_L/xi_R` geometry parameters
  and no Eq.(12) branch.
- Independently validate continuous/discrete dynamics, parameter sensitivity, Jacobians, generated
  artifact parity, solver audit and resource bounds before controller integration.
- Derive and validate per-side longitudinal wheel-origin relative acceleration as an affine function
  of the existing 12D reduced generalized acceleration.
- Add an opt-in Phase34 WBC profile containing only common/differential longitudinal wheel tracking
  on top of Phase27 Minimal; keep the planner common channel and a reset-time differential hold.
- Run a gain-free 2x2 authority/wrench-preservation screen before freezing at most three gain sets.
- Re-run the frozen Phase29 T0/T1 corrective oracles with the 12D solver, then static and bounded
  straight/turning closed-loop cases only if earlier gates pass.
- Preserve append-only failed/inconclusive evidence and finish with regression, REVIEW, and RECORD
  only if all gates pass.

## Out of Scope

- Slack-aware NMPC/WBC, feasibility or slack feedback, workspace feedback, predictive wheel-workspace
  constraints, or changing the existing slack formulation/weight.
- Any NMPC state augmentation, x18/x24 candidate, hidden-state estimator, wheel spin/mesh phase state,
  constrained-dynamics closure repair, or restoration of Eq.(12).
- Retuning `Q/Qe/R`, adding `Delta u`, changing terminal structure, horizon, bounds, `Ts`, integration,
  solver family, warm-start lifecycle, or input semantics.
- Restoring base-X, height, attitude, leg, Phase33 zeta, rolling or other WBC tasks; changing 42
  variables, 104 hard rows, contact cones, torque/acceleration limits, ProxQP, or wrench/slack weights.
- Adding a differential motion planner, changing the common planner limits/dynamics, or widening the
  production `0.20 m/s` / `0.08 rad/s` motion contract.
- Smooth-wheel collision revision, terrain, identified/CAD profile, hardware, real-machine or
  production-safety approval.
- Using a Phase34 failure to implement any fallback in the same Phase. A failed gate stops downstream
  execution and records the failure class.

## Frozen Decisions

- **Authority/non-overwrite:** Phase27 RECORD/formal-v2, Phase29 RECORD/formal replays and current live
  source are inputs. All Phase34 model, generated artifacts, configs, runners and evidence use new
  names and refuse nonempty output. Phase27/29 artifacts and production controller selection remain
  byte/behavior invariant.
- **State:** `x12=[p_N(3), r_rel(3), v_N(3), omega_N(3)]`, with the same yaw-aligned
  `R=Exp(r_rel)R_ref` chart and `0.35 rad` limit as Phase27. NMPC reference contains only these twelve
  values.
- **Input:** `u12=[F_L^B,tau_L^B,F_R^B,tau_R^B]`, wheel-on-body internal wrench at each wheel-body
  origin, body/FLU, Phase27 order/sign/units. Requested and algebraic-realized wrench remain distinct
  logged quantities.
- **Parameters:** `p=[R_ref(9),xi_L,xi_R]`. The measured `xi` pair is finite, within current per-side
  workspace, sampled at the solve tick and held constant at all 21 OCP nodes. It is geometry only and
  is never optimized, propagated, penalized or replaced by feedback from slack/workspace.
- **Base model:** reuse the first twelve Phase27 equations and body-only mass/COM/inertia. Preserve
  wheel-origin y/z and measured-x moment arms. Delete only the four wheel derivative rows; no Phase23
  locked-composite mass/model and no Eq.(12) term enter the candidate.
- **OCP:** actual Phase27 v2 first-12 state cost, input cost/equilibrium/bounds, first-12 state envelope,
  terminal multiplier `10`, `20 ms/N=20/two 10 ms RK4`, SQP-RTI/HPIPM and production lifecycle are
  fixed. There is no wheel-state cost/bound/reference and no input-rate cost.
- **Planner/WBC reference:** existing common planner position, velocity and acceleration go directly
  to the Phase34 WBC reference. At reset,
  `xi_delta_ref=(xi_R-xi_L)/2`; thereafter it is held with zero differential velocity and acceleration
  feedforward. No new differential trajectory generator is implied.
- **WBC coordinate:** per side `xi_i=e_x^T R_BN(p_wheel_i-p_base_control)`, matching the current
  `NominalWbcModel` measurement. The analytic contract is
  `ddxi_i=A_xi_i(q,nu)*nudot+b_xi_i(q,nu)`. Common/differential rows are the exact half-sum and
  half-difference; planner acceleration is retained as feedforward.
- **Task isolation:** the opt-in Phase34 profile starts from Phase27 Minimal and adds only the two
  longitudinal soft rows. Dynamics, hard constraints, contact task, internal-wrench/slack task,
  dimensions, scaling and production profiles remain unchanged. Gain selection is forbidden until
  direct-acceleration authority passes.
- **Test range/order:** component -> gain-free WBC -> limited gains -> T0/T1 offline -> static
  closed loop -> `+/-0.1,+/-0.2 m/s` -> `+/-0.08 rad/s` turning. Downstream tests do not run after a
  blocking gate.
- **Interpretation:** Phase34 PASS means only that this architecture is a viable opt-in candidate on
  the current nominal MuJoCo profile and frozen cases. It does not prove x12 Markov closure of the
  full plant, predictive wheel-workspace safety, terrain robustness, or real-machine readiness.

## Open Questions / Decision Gates

- **DG34-00 / CLOSED PASS / audit authority:** the live architecture and conflicts above are
  grounded. Closing this gate authorizes only the PLAN, not implementation or feasibility.
- **DG34-01 / 12D model:** independent continuous/discrete oracle, finite differences, left/right and
  common/differential wrench cases, `xi` parameter sensitivity and generated-model parity meet
  thresholds frozen in T02. Eq.(12) and wheel derivatives are absent. Failure -> `P34-A` and stop.
- **DG34-02 / 12D OCP:** dimensions, fields, costs, constraints, lifecycle and generated artifact are
  identical to the frozen projection contract; equilibrium/dynamic corpus, RTI/converged audit and
  deadline pass. Failure -> `P34-B` and stop.
- **DG34-03 / longitudinal WBC authority:** independent `ddxi` affine parity passes; direct
  common/differential acceleration perturbations have correctly signed, adequately conditioned 2x2
  authority while preserving hard feasibility and requested-wrench realization under predeclared
  thresholds. Failure -> `P34-C` or `P34-D` and stop before gains.
- **DG34-04 / wheel tracking:** at most three predeclared gain sets are screened. One must track small
  common step/ramp plus differential hold without workspace/contact/hard/deadline failure or excessive
  wrench/slack/torque degradation. Failure -> `P34-E` and stop.
- **DG34-05 / corrective action:** authoritative Phase29 T0 and T1 production-prefix/cold/converged
  inputs are projected without changing their first twelve state/reference values. Both must meet the
  existing restorative-direction definitions and base-group non-regression gates. Results may not be
  described as proving a unique cause because removing states also removes their costs/constraints.
  Failure -> `P34-F` and stop.
- **DG34-06 / static closed loop:** T0 hold satisfies frozen safety/contact/base/wheel/wrench/slack/
  torque/QP/resource gates and fresh replay. Failure -> `P34-G` and stop.
- **DG34-07 / bounded motion:** straight cases within `|v|<=0.20 m/s`, followed by left/right
  `|yaw rate|<=0.08 rad/s`, satisfy frozen safety, tracking, differential-drift, wrench/slack, contact,
  solver and deadline gates. Failure -> `P34-H`; no differential planner or retuning is added.
- **DG34-08 / terminal decision:** all gates and production-regression/fault/non-overwrite checks pass
  -> `P34-I_architecture_feasible_current_nominal`; otherwise REVIEW=REWORK with the first truthful
  failure class. Default production remains Phase27 in either outcome.

Numerical thresholds, perturbation corpus, seeds, run durations and stop precedence are frozen in
T02 before any candidate result is visible. Existing Phase27/29 thresholds are reused where their
metric semantics match; new WBC tracking/authority thresholds require explicit prefreeze evidence.

## Interfaces and Compatibility

- Input: canonical `RobotState`; base-only motion reference; existing common planner target; measured
  per-side `xi/dxi`; unchanged 12D wheel-origin internal-wrench request.
- Internal flow: `RobotState -> x12 + measured xi parameters -> 12D NMPC -> requested internal wrench`;
  in parallel `RobotState -> existing common planner -> Phase34 WBC wheel reference`; both feed one
  opt-in full-body WBC solve.
- Output: unchanged six-joint `TorqueCommand`; additive Phase34 diagnostics may expose requested/
  realized wrench, slack, `xi/dxi/ddxi`, common/differential errors and solver/resource metrics.
- Must preserve: public RobotState/TorqueCommand schemas, frames/sign/order, 42D/104-row hard WBC,
  ProxQP, Phase27 timing/fault/fail-zero/reset behavior, current motion limits and all non-Phase34
  controller modes.
- Allowed only during later Phase34 execution: namespaced 12D model/solver/artifact, `xi` model
  parameters, longitudinal affine model rows, one opt-in WBC/controller profile, additive diagnostics,
  tests/runners/configs and append-only evidence.

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| T01 | Audit design input against live code and frozen evidence | proposal, CBM/source, Graphify, Phase27–33 | Design Audit, reuse/non-reuse map, conflicts and scope boundary | CBM coverage + direct reads; no product implementation | done |
| T02 | Freeze method, corpus, thresholds and evidence schema | T01, Phase27/29 methods | versioned config/schema, hashes, failure precedence and non-overwrite rules | dependency probe, schema synthetic checks; values fixed before candidate runs | done |
| T03 | Build independent 12D parameterized base model oracle | Phase27 model, measured `xi`, T02 | model contract and oracle corpus | DG34-01 continuous/discrete/FD/parameter/sign/symmetry gates | done |
| T04 | Generate and wrap append-only 12D acados OCP | T02/T03, Phase27 v2 config/generator | generated artifact, offline wrapper, equivalence manifest | DG34-02 parity, solver corpus, RTI/SQP, audit/deadline/reset | done |
| T05 | Derive and validate longitudinal WBC kinematics | current NominalWbcModel, independent MuJoCo/FD oracle | `xi/dxi/ddxi` contract and per-side affine rows | coordinate/sign/rotating-frame/bias/FD parity | done |
| T06 | Add isolated Phase34 WBC profile and run gain-free authority | T02/T05, Phase27 Minimal | opt-in two-row task and 2x2 authority corpus | A/lower/upper/dimensions invariant; DG34-03 wrench/hard/conditioning | done |
| T07 | Freeze and screen at most three tracking gain sets | T06, planner output and differential hold | gain manifest and small step/ramp tracking evidence | DG34-04; no post-result gain/threshold changes | done |
| T08 | Re-run frozen T0/T1 corrective oracles with x12 OCP | T04, Phase29 snapshots/prefixes | projected lifecycle/cold/converged reports | DG34-05 restorative direction and base-group regression | blocked |
| T09 | Integrate the opt-in planner-to-WBC candidate and run static hold | T04/T07/T08, current runner | namespaced controller path and T0 closed-loop evidence | DG34-06 safety/contact/wheel/wrench/slack/torque/QP/resource/replay | blocked |
| T10 | Run bounded straight and turning cases | T09, current motion contract | `+/-0.1,+/-0.2 m/s` and `+/-0.08 rad/s` corpus | DG34-07; common/differential drift and all closed-loop gates | blocked |
| T11 | Run fault, deadline, regression and non-overwrite audit | T03–T10 | fresh replay, regression report and REVIEW input | DG34-08; build/test, Phase27/29 parity, failure-path checks | blocked |

Task status is `todo / doing / done / blocked`.

## Validation Plan

### Automated

- Repository Python experiments use `./.venv/bin/python`; dependency/version probe and `py_compile`
  precede stable formal output.
- Later execution adds focused C++ component tests for x12 model/OCP, longitudinal affine map, WBC
  algebra/profile isolation, planner reference routing and fault/reset behavior.
- Generated CasADi/acados dynamics are compared against the independent oracle for next state,
  state/input/parameter Jacobians; solver outputs receive independent defect, bound and stationarity
  audits.
- From `ros_ws/`, Release `colcon build` and `colcon test` must pass; focused test output, dependency
  versions and commands are recorded in Phase34 evidence.
- Formal and replay runners refuse overwrite and store model/config/source/generated/input hashes,
  interpreter/dependencies, seeds, exact commands, invalid/superseded relation and output hashes.

### Manual / Evidence

- Compare Phase34 projected T0/T1 inputs and first-twelve state/reference values against Phase29
  authority before reading corrective results.
- Inspect per-case time series for base pose/twist, per-side/common/differential wheel coordinate,
  planner reference, requested/algebraic-realized wrench, signed slack, torque margins, contact,
  constraints, solver residuals and timing.
- Any environment or dependency failure is recorded separately and is not model/control evidence.
  Failed, inconclusive and superseded numerical runs remain append-only.

## Acceptance Criteria

- [x] DG34-01 and DG34-02 establish the exact Phase27-semantics x12 model/OCP without Eq.(12).
- [ ] DG34-03 and DG34-04 establish independent longitudinal WBC authority and bounded tracking while
      preserving wrench/hard constraints.
- [ ] DG34-05 establishes restorative T0 and T1 actions without Q/Qe/R/terminal retuning.
- [ ] DG34-06 and DG34-07 pass static, bounded straight and bounded turning evidence.
- [ ] Fault/deadline/reset/replay/regression/non-overwrite checks pass and production Phase27 remains
      unchanged.
- [ ] REVIEW is PASS before RECORD is created; otherwise REVIEW is REWORK and the first failed gate is
      recorded without implementing a fallback.

## Execution Notes

Execution preserved the Phase33 worktree and kept the vertical zeta profile separate. DG34-01,
DG34-02 and DG34-03 passed. T07 then froze exactly three PD gain sets before running six current-
nominal MuJoCo step/ramp cases. All six were rejected by the unchanged WBC workspace contract at
ticks 90--92; none met the `1 mm / 0.2 s` common tracking gate. Per the frozen order, T08--T11 were
not executed and no ControllerCore/production selection was added.

## Blockers

DG34-04: `P34-E_wheel_tracking_failure`. All three predeclared gain sets failed both step and ramp;
execution stops before corrective-action and integrated closed-loop tests.
