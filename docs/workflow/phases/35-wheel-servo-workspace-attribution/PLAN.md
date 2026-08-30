# Phase 35: Wheel-Position Servo Workspace Failure Attribution — PLAN

Status: `complete`

Design input: user-provided Phase35 attribution proposal. It is accepted after grounding corrections
below. Current live source and append-only Phase34 evidence remain authoritative. This Phase diagnoses
the first physical mechanism behind `NominalWbcModel::kOutsideWorkspace`; it does not repair it or
decide the final feasibility of `12D base NMPC + wheel planner + WBC xi tracking`.

## Goal

Reproduce and causally attribute the Phase34 wheel-servo workspace rejection by identifying:

```text
first sustained physical change
  -> coordinate/geometric mode
  -> signed live-workspace margin loss
  -> exact canonical joint/side/bound
  -> kOutsideWorkspace
```

The terminal result must distinguish pre-existing Minimal-WBC drift, zero-command task activation,
direct longitudinal-acceleration coupling, zero-displacement position-servo coupling, commanded
tracking motion, upstream wrench/contact/torque loss, and a validity-contract issue.

## Design Audit and Corrections

- **The live workspace gate is much narrower than the design input's candidate list.** The only
  `kOutsideWorkspace` return is in `NominalWbcModel::evaluate`, before passive closure reconstruction.
  It checks six canonical actuated joint deltas from their frozen equilibrium values. It does not
  directly check xi, zeta, wheel-origin xyz, passive coordinates, closure residual, Jacobian rank,
  leg reach, base pose or contact. Those quantities may be causal predictors, but cannot be reported
  as the exact triggering condition.
- **Exact trigger map:** canonical order is
  `[left_hip,left_knee,left_wheel,right_hip,right_knee,right_wheel]`; side is independent; bounds on
  `q_canonical-q_equilibrium` are hip `[-0.65,+0.65] rad`, knee `[-0.75,+0.75] rad`, wheel
  `[-1,+1] rad`. The test is strict outside (`< lower` or `> upper`); equality is admitted.
- **Bound meaning:** the constants are exported from Phase21 `workspace_rad` validation samples.
  They are the validated runtime-model domain, not proven mechanical joint limits. Crossing a bound
  therefore proves departure from the frozen model contract, not by itself that the physical plant
  is unreachable or that the contract is wrong.
- **Wheel spin is a first-class candidate:** the live gate includes canonical wheel joint position.
  A wheel-angle bound can be first even when wheel-origin xi remains geometrically admissible. Because
  the current MuJoCo collision is a rotating mesh, wheel spin/mesh phase must be logged separately;
  neither “internal leg geometry drift” nor “over-conservative bound” may be inferred from the status.
- **Phase34's failing runner did not run x12 NMPC.** It used the current nominal MuJoCo reset,
  Phase27 Minimal wrench semantics, a fixed Phase27 equilibrium interaction-wrench request, the
  Phase34 xi WBC profile and planner/PD reference. Phase35 controls reuse that exact causal surface.
  A production Phase27 NMPC hold would change the wrench lifecycle and is not the H0 control.
- **Task activation and position feedback must be separated.** “xi profile with zero initial error”
  becomes two controls: an enabled xi row with desired acceleration identically zero, and a closed
  PD hold at reset xi. The latter reacts once any error appears and is not a zero-command task.
- **Direct acceleration must be bounded.** Separate constant accelerations would intentionally build
  unbounded velocity. Phase35 uses sign-reversed, zero-net-velocity pulse pairs, fixed before results,
  so any lasting hidden-coordinate/margin change is interpretable.
- **Paper comparison is downstream interpretation only.** The repository's reproduced Simulink
  baseline and the source-paper task vocabulary are not assumed identical. T09 must cite the exact
  reproduced source/equation for any mapping and must not claim that an unnamed paper task regulates
  the observed Phase35 coordinate.
- **Workflow correction:** only this PLAN is created while the Phase is planned. REVIEW and the
  requested attribution reports are execution deliverables; RECORD remains forbidden unless REVIEW
  is PASS.

Grounding uses CBM project `W_L_ws`, generation `2026-08-29T06:47:42Z` (Verify tier). The relevant
WBC sources have changed metadata from Phase33/34 and were therefore read directly after graph
discovery. `docs/`, configs, tools and Phase evidence are excluded from the main index and were read
directly. Existing Graphify history was used only for Phase21/27/34 and reproduced-baseline task
relationships; current source owns the exact live gate.

## Current State

- Phase34 DG34-01/02 passed the append-only Phase27-semantics x12 model and offline OCP. No Phase34
  ControllerCore mode or runtime x12 solver selection exists.
- DG34-03 passed local physical longitudinal authority: minimum self gain `0.7009409313`, maximum
  cross/self `0.0130391903`, maximum condition number `1.350769849`, maximum realized-wrench change
  `0.1722%`, and maximum hard violation `1.37e-8`.
- DG34-04 froze three gains (2.5/3.5/5 Hz) and six step/ramp runs. All exited at ticks 90--92 with
  model status `kOutsideWorkspace`; final available common error was `4.195--9.580 mm`.
- Before exit, bilateral contact, hard feasibility, WBC deadline, normalized slack, differential
  drift and torque limits passed. The Phase34 CSV did not log canonical workspace margins or the
  invalid tick's full state, so it cannot identify which live bound triggered.
- Production selection remains Phase27; Phase34's diagnostic xi profile is opt-in only.

## Scope

- Audit every live `kOutsideWorkspace` path and the provenance/semantics of its bounds.
- Add one behavior-invariant workspace inspector used by both `NominalWbcModel::evaluate` and the
  offline Phase35 runner, preventing duplicated gate logic.
- Add append-only, offline full-envelope logging that persists the final rejecting sample.
- Execute ordered hold, zero-command task, direct-acceleration pulse, zero-displacement PD-hold and
  exact Phase34 tracking controls.
- Determine first mover and event precedence with predeclared margin/trend semantics.
- After a physical direction is identified, compare it structurally with explicitly sourced paper/
  reproduced-baseline WBC tasks and recommend exactly one next experiment.

## Out of Scope

- Any x12/x16/x18/x20/x24 NMPC model, state, OCP, cost, terminal, horizon, timing, solver, lifecycle,
  constraint, input or reference change; any Eq.(12) restoration.
- Planner dynamics, limits, workspace or reference changes; Phase34 gain/threshold/task-scale tuning,
  a fourth gain, or feedforward removal/addition.
- Slack/workspace/feasible-wrench feedback; wrench/slack weights or formulation changes.
- Contact model/task, friction cone, wheel collision geometry, torque/acceleration limits, ProxQP,
  42 variables or 104 hard rows.
- Adding/restoring zeta, height, pitch, translation, rotation, posture, rolling-speed or any other
  WBC stabilization task.
- Terrain, smooth-wheel, hardware or real-machine tests; bypassing or weakening a workspace bound.
- Implementing the repair or declaring the overall x12 architecture feasible/infeasible.

## Frozen Decisions

### Authority and non-overwrite

- Phase34 model/OCP/authority results remain PASS inputs; DG34-04 remains the authoritative failed
  tracking corpus. Phase35 uses new names and output directories and never overwrites Phase34 data.
- Production Phase27 and all non-Phase35 controller modes must remain byte/behavior invariant.
- Phase35 does not invoke x12 NMPC; all dynamic attribution runs use the exact Phase34 reset, current
  nominal scene, `2 ms` physics / `10 ms` WBC schedule and fixed equilibrium interaction wrench.

### Workspace contract

For canonical index `j`:

```text
q_eq[j] = kCanonicalOffset[j] - kEquilibriumActiveNative[j]
delta[j] = RobotState.joint_position_rad[j] - q_eq[j]
lower_margin[j] = delta[j] - lower[type(j)]
upper_margin[j] = upper[type(j)] - delta[j]
signed_margin[j] = min(lower_margin[j], upper_margin[j])
```

`type(j)=j mod 3` maps to hip/knee/wheel. The first failed index is the first canonical loop index
whose signed margin is negative, matching live evaluation order. The inspector reports all margins,
the minimum-margin index, first failed index, side, joint type, raw state, equilibrium, delta, bound
and signed margin. `evaluate` consumes this inspector without changing acceptance behavior.

### Diagnostic contract

Each WBC tick, including the rejecting tick, records:

- run/case/gain/profile/tick/time and reset/virtual-onset markers;
- six canonical q/dq, q-equilibrium deltas and both workspace margins;
- raw MuJoCo active/passive joint q/dq and wheel spin/mesh phase;
- per-side wheel-origin relative xyz and velocity xyz in body/FLU; xi/zeta and common/differential
  position/rate channels;
- base position, quaternion/rotation-vector, linear velocity and angular velocity;
- planner xi/dxi/ddxi, final desired per-side/common/differential xi acceleration and physical ddxi;
- requested/algebraic-realized wrench, signed slack, wrench residual;
- torque and per-joint torque margins, bilateral contact and normal load;
- model/QP status, hard/primal/dual/stationarity residuals, iterations and solve time.

Raw MuJoCo geometry is logged independently so the rejecting tick remains observable even though the
WBC model returns before closure reconstruction. Derived quantities always retain their raw inputs.

### Experiment corpus

Every case is 150 WBC ticks (`1.5 s`). Tick 50 (`0.5 s`) is the virtual/actual activation time.
All controls start from the same state and fixed wrench as Phase34.

1. `H0_minimal_hold`: Phase27 Minimal WBC profile, no xi row, unchanged fixed wrench.
2. `H1_zero_ddxi_row`: Phase34 xi profile with desired longitudinal acceleration identically zero;
   no planner/PD feedback.
3. `D_positive/D_negative`: only if H0/H1 remain admissible through 1.5 s. Apply common desired
   acceleration `s*0.25 m/s^2` for 0.10 s, `-s*0.25 m/s^2` for 0.10 s, then zero, with `s=+1/-1`;
   differential desired acceleration stays zero. This pulse has zero commanded final velocity and
   nominal `2.5 mm` common displacement.
4. `H2_g1/g2/g3_pd_hold`: only after H0/H1/direct controls remain admissible. Use each frozen
   Phase34 gain with common/differential references fixed at reset measurements and planner target
   unchanged; planner feedforward remains as implemented.
5. Exact Phase34 six-run replay: 2.5/3.5/5 Hz crossed with the frozen 5 mm step and
   `0.02 m/s * 0.25 s` ramp, without any numerical or lifecycle change.

If an upstream control reproducibly fails, later controls may run only when required to distinguish
the first mover; their outcomes cannot supersede the earlier causal event.

### Trend and precedence semantics

- Normalize each signed workspace margin by that joint type's half-width.
- `first_sustained_margin_loss` is the earliest five-tick window with at least four negative
  step-to-step changes and total normalized loss `>=1e-3`.
- `near_boundary` is normalized signed margin `<=0.05`; failure is signed margin `<0`.
- Two events within one WBC tick (`10 ms`) are simultaneous; an upstream precedence claim requires
  the candidate event to begin at least two ticks earlier and reproduce in a fresh run.
- First mover is the earliest sustained physical trend that precedes and quantitatively predicts the
  limiting margin. Merely having the largest final excursion is insufficient.
- For the limiting coordinate report reset, activation, trend onset, near-boundary, last-valid and
  rejecting values; both margins, normalized margin and loss rate; and `Delta y / Delta xi_common`
  where the denominator is material. Common/differential and left/right modes remain separate.

### Interpretation and stop rule

- `kOutsideWorkspace` cannot be relabeled as xi authority loss, zeta drift, singularity, contact loss
  or invalid contract without the corresponding earlier evidence.
- A wheel joint bound is reported as wheel spin/mesh-phase validity loss, not leg reach loss.
- A bound's sampled-validation provenance is insufficient to classify `P35-I`; an independent
  validity audit is required, without bypassing the gate in closed loop.
- Once one mechanism reproduces and its nearest counterfactual excludes earlier alternatives,
  downstream repair-oriented experiments stop. Unrun later branches are recorded as causally
  ineligible, not silently treated as PASS.

## Decision Gates

- **DG35-00 / source authority:** exact return paths, joint mapping, expressions, bounds, provenance
  and behavior-invariant inspector parity close. Failure -> `P35-U` and stop.
- **DG35-01 / evidence completeness:** the final invalid sample and every frozen diagnostic field are
  finite/available when semantically defined; derived/raw and live-inspector parity pass. Failure ->
  `P35-U` and stop before causal runs.
- **DG35-02 / hold controls:** H0 and H1 produce fresh deterministic replay. H0 failure closes
  `P35-A`; H0 stable/H1 failure closes `P35-B`. Otherwise direct pulses are authorized.
- **DG35-03 / direct acceleration:** sign-mirrored zero-net-velocity pulses distinguish signed
  longitudinal coupling. Reproducible hidden-coordinate/margin drift absent in H0/H1 closes
  `P35-C`; otherwise PD holds are authorized.
- **DG35-04 / zero-displacement servo:** frozen-gain H2 holds determine whether feedback closure alone
  creates the drift. A reproducible failure absent from prior controls closes `P35-D` or, if only one
  gain differs, candidate `P35-J`; otherwise motion replay proceeds.
- **DG35-05 / tracking replay:** all six exact Phase34 cases reproduce status/timing within frozen
  deterministic tolerances and expose the exact trigger and first mover. Non-reproduction -> `P35-K`
  or `P35-U`, not a repair.
- **DG35-06 / precedence:** workspace, wrench, slack, torque, contact, hard feasibility and QP events
  are ordered. Earlier wrench loss -> `P35-G`; earlier contact/torque loss -> `P35-H`; otherwise the
  supported geometry/base/internal class is retained.
- **DG35-07 / interpretation:** the identified direction is mapped only to an explicitly sourced
  paper/reproduced-baseline task, or recorded as having no justified mapping. Exactly one next
  experiment is selected; no repair is implemented.
- **DG35-08 / terminal attribution:** exact live condition, physical driver, activation branch,
  corpus reproduction and precedence form one reproducible causal chain. PASS permits REVIEW and
  RECORD; ambiguity yields REVIEW=REWORK with the first unresolved evidence gap.

## Attribution Classes

Use the first supported class:

```text
P35-A_pre_target_minimal_wbc_workspace_drift
P35-B_zero_ddxi_task_activation_workspace_drift
P35-C_direct_longitudinal_acceleration_hidden_coordinate_drift
P35-D_xi_position_loop_hidden_coordinate_drift
P35-E_specific_internal_geometry_drift
P35-F_base_state_drift_drives_workspace_loss
P35-G_wrench_realization_loss_precedes_workspace_failure
P35-H_contact_or_torque_limit_precedes_workspace_failure
P35-I_workspace_contract_or_validity_gate_issue
P35-J_gain_specific_servo_instability
P35-K_no_reproducible_workspace_failure
P35-U_unresolved
```

`P35-E/F` may refine B/C/D but cannot replace the activation class; `P35-G/H` take precedence when
their event demonstrably begins first. Any new subclass requires direct logged evidence and a frozen
definition before formal classification.

## Interfaces and Compatibility

- Input: canonical `RobotState`, unchanged fixed Phase27 equilibrium interaction wrench, existing
  planner output and Phase34 diagnostic xi acceleration reference.
- Diagnostic addition: one read-only workspace-inspection result and append-only offline CSV/JSON;
  no public RobotState/TorqueCommand schema change.
- Output: unchanged six-joint `TorqueCommand`; Phase35 runner adds only evidence fields.
- Preserve: all controller modes, profiles, QP dimensions/hard rows, WBC acceptance/status ordering,
  solver warm start, torque sign/order and current nominal MuJoCo state/reset semantics.

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| T01 | Audit live workspace contract and bound provenance | CBM/source, Phase21 runtime profile | `workspace-contract.md` | exhaustive return-path/mapping/bound audit + coverage | done |
| T02 | Freeze method, corpus, trend/precedence and schema | T01, Phase34 runner/evidence | versioned config, `state-envelope-contract.md`, hashes | schema synthetic cases; values frozen before dynamic results | done |
| T03 | Implement behavior-invariant inspector and full logger | T01/T02 | diagnostic API, runner/log schema | exact old/new acceptance parity; invalid-tick persistence | done |
| T04 | Run H0 Minimal and H1 zero-ddxi-row holds | T03 | `hold-attribution.md`, formal/replay | DG35-02, 1.5 s or earlier exact rejection | done |
| T05 | Run sign-mirrored direct-ddxi pulse controls | T04 PASS-to-proceed | `direct-ddxi-attribution.md` | DG35-03; zero-net-velocity and sign symmetry | blocked |
| T06 | Run frozen-gain zero-displacement PD holds | T05 PASS-to-proceed | hold-servo evidence | DG35-04; no target displacement or gain change | blocked |
| T07 | Replay exact Phase34 step/ramp corpus | T03–T06 | `tracking-replay.md` | DG35-05; six cases, fresh replay, original semantics | done |
| T08 | Perform first-mover/coupling/precedence analysis | T04–T07 | `first-mover-analysis.md` | DG35-06; timing, margins, common/diff, raw/derived | done |
| T09 | Compare identified direction to sourced WBC task coverage | T08 | `paper-wbc-coverage-comparison.md` | exact source/equation; no invented task or repair | done |
| T10 | Select one next experiment and review | T01–T09 | `next-direction.md`, `REVIEW.md` | DG35-07/08; build/test/replay/non-overwrite | done |

Task status is `todo / doing / done / blocked`.

## Required Execution Evidence

Execution creates append-only evidence containing config/source/runner/scene/initial-state hashes,
exact command, interpreter/dependency versions, run and replay IDs, workspace condition/index/side/
joint/value/bound/margins, first-failure tick, full time-series path and supersedes/replay metadata.
Repository Python uses `./.venv/bin/python`; dependency probe and `py_compile` precede stable output.
Release `colcon build/test` runs from `ros_ws/`.

Planned reports are:

```text
workspace-contract.md
state-envelope-contract.md
hold-attribution.md
direct-ddxi-attribution.md
tracking-replay.md
first-mover-analysis.md
paper-wbc-coverage-comparison.md
next-direction.md
REVIEW.md
```

Only PLAN exists before execution. No RECORD is created unless the terminal attribution REVIEW is
PASS.

## Acceptance Criteria

- [x] The exact first live condition, canonical index, side, joint type, raw value, bound and signed
      margin are reproduced without changing the gate.
- [x] The earliest physical driver and activation branch are isolated by the nearest ordered
      counterfactual; every skipped branch is explicitly causally ineligible.
- [x] The Phase34 six-case mechanism reproduces, or non-reproduction is classified truthfully.
- [x] Wrench/slack/contact/torque/hard/QP precedence is established to one-tick resolution.
- [x] Paper/reproduced-baseline comparison cites an actual task/equation or states no supported map.
- [x] Exactly one next experiment is recommended; no repair, retuning, feedback or architecture
      change is implemented.
- [x] Production Phase27 and prior formal artifacts remain invariant; fresh replay and repository
      regression pass.

## Required Terminal Form

```text
Phase34 wheel-position servo
  -> <first causal physical change>
  -> <identified coordinate / geometry>
  -> <workspace margin degradation>
  -> <exact canonical side/joint/bound>
  -> kOutsideWorkspace

NMPC status: <not implicated / implicated with evidence>
Wrench realization: <healthy / degraded and timing>
Contact/torque/hard constraints: <healthy / degraded and timing>
Recommended next experiment: <one experiment only>
```

## Blockers

None for planning. Execution requires explicit activation and must begin with T01/T02; no dynamic
result may be interpreted before the workspace inspector and evidence schema pass.
