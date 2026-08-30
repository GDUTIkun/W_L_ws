# Phase 33: Low-Dimensional Closure Recovery via WBC Manifold Regulation — PLAN

Status: `review`

Design input: user-provided “Phase 33: Low-Dimensional Closure Recovery via WBC Manifold
Regulation”, accepted with two authority corrections. This Phase can establish approximate closure
only on a WBC-regulated slow manifold; it cannot turn the unregulated MuJoCo plant into an exact
16-state Markov system. Phase32 “realized wrench” is the WBC algebraic internal-wrench realization,
not a directly measured physical MuJoCo joint wrench.

## Goal

Determine whether an isolated two-side wheel-height/normal-velocity WBC task can rapidly attract the
full plant to a low-dimensional manifold while preserving internal-wrench realization, thereby
recovering approximate `x16 + W^I -> ddxi` closure over the NMPC-relevant time scale.

## Current State

- Phase32 proved `P32-C/M5` on the current floating-base mesh-contact plant: C1 `0.1083278971`, C2
  `2.100656939`, C3 `1.675359928 m/s^2`; wheel mesh phase produced `1.141498–1.32688 m/s^2`.
- The current Minimal WBC is 42-variable/104-hard-row ProxQP, preserves its iterate across ticks, and
  already includes a six-row soft contact-acceleration task plus the internal-wrench task.
- The 16-state projection omits leg normal coordinates/rates and wheel spin/phase. Phase32 showed one
  leg-coordinate null direction per side is removed locally by wheel height, but `x24` is only a
  necessary observable superset, not an approved design.
- Production remains the Phase27 Minimal profile and Phase27 NMPC artifact.

## Scope

- Reproduce Phase32 C1/C2/C3, mesh-phase, projection, wrench and 42D/104-row authorities.
- Freeze body/FLU wheel-origin relative vertical coordinates `zeta_L/R`, their rates and nominal
  geometric reference.
- Add one opt-in diagnostic WBC profile containing only two soft `ddzeta` objective rows on top of
  Phase27 Minimal; preserve all variables, hard rows, bounds, contact and wrench contracts.
- Verify the analytic `ddzeta = A_zeta nudot + b_zeta` expression independently.
- Run a gain-free acceleration-authority screen before selecting at most three gain sets.
- If preceding gates pass, retest C1/C2, bandwidth, C3, smooth round-wheel contact, minimum state,
  rollouts and original Phase29 corrective oracles in that order.

## Out of Scope

- NMPC Q/Qe, terminal cost, reference, horizon, solver lifecycle or state dimension changes.
- `x24` implementation; base-X, height, attitude, leg-posture, wheel-position or wheel-spin tasks.
- Production safety-envelope, internal-wrench contract, contact hard rows or wrench weight changes.
- Treating a smooth round-wheel collision revision as equivalent to the current mesh plant without
  its own geometry/contact validation.

## Frozen Decisions

- `zeta_i = e_z^T R_BN (p_wheel_i - p_base_control)`, positive body/FLU up; side order left/right.
- `dzeta` is the time derivative in the rotating body frame. `zeta_ref` comes from the frozen nominal
  equilibrium state, is constant, and never follows NMPC tracking error.
- Candidate task: `ddzeta_des = -kp(zeta-zeta_ref)-kd*dzeta`, soft objective only.
- Gain-free Gate4 uses direct `ddzeta_des` perturbations at a predeclared `1 m/s^2` residual scale;
  this normalization is not selected from closure outcomes.
- Internal-wrench tracking remains primary. Requested wrench stays identical; algebraic realized
  wrench, signed slack, hard violation, torque margin and contact state are recorded separately.
- The Phase33 profile shares Phase27 Minimal wrench semantics. `kNominal` and `kPhase27Minimal`
  outputs must remain invariant.
- Smooth contact is an append-only diagnostic plant revision with matched mass, radius, inertia and
  friction. It is not eligible for production in this Phase.

## Decision Gates

- **DG33-00 Authority:** current Phase32 authorities and 42D/104-row contract reproduce. Any mismatch
  stops task design.
- **DG33-01 Coordinate:** analytic `zeta/dzeta` and affine `ddzeta` match independent MuJoCo/finite
  difference checks; task is demonstrably distinct from existing contact-normal rows.
- **DG33-02 Algebra:** Phase33 changes H/g only; A/lower/upper, dimensions and Minimal outputs are
  unchanged. Non-finite references fail closed.
- **DG33-03 Authority:** centered direct `ddzeta_des` perturbations have finite, correctly signed
  self-channel response with gain `>=0.2`, cross-side gain `<=0.5` of self, no hard violation, and
  algebraic realized-wrench relative change `<=2%`. Otherwise P33-A/B and stop.
- **DG33-04 Gains:** at most G1/G2/G3, preselected from 10 ms sampling, local authority and margins.
- **DG33-05 C1/C2:** both same-x16/same-request closure differences `<0.05 m/s^2`; realized-wrench
  pair mismatch `<=2%`, bilateral contact and hard gates pass.
- **DG33-06 Bandwidth:** settling time is `<=80 ms` and at least 5x faster than the 0.4 s horizon;
  no overshoot outside the frozen local validity range and wrench/hard gates remain valid.
- **DG33-07 C3/contact:** report C3 separately. Smooth round wheel must first pass matched-plant
  geometry/contact gates before its angle/spin evidence is admissible.
- **DG33-08 Minimum state:** decide L0 x16, L1 x18, evidence-minimal normal augmentation, or L3; no
  dimension is selected by preference alone.
- **DG33-09 Rollout/corrective:** only after closure; 20/40 ms normalized wheel-rate error `<0.1`,
  then 100/200/400 ms and original Phase29 T0/T1 with no base-group regression.
- **DG33-10 Production:** only after all component, closure, bandwidth, contact, rollout, corrective,
  RTI/SQP, fault, deadline and regression gates PASS.

## Interfaces and Compatibility

- Input: canonical `RobotState`, unchanged 12D internal-wrench request, and diagnostic constant
  `zeta_ref`, `kp`, `kd`/direct acceleration target.
- Output: unchanged six-joint `TorqueCommand`; diagnostic `zeta/dzeta/ddzeta`, residuals and margins.
- Must preserve: 42 variables, 104 hard rows, 12 dynamics equalities, ProxQP, contact-centered wrench,
  Phase27 Minimal and Nominal behavior.
- Allowed: one opt-in enum profile, two reference acceleration values, model diagnostic coordinate
  and affine rows, offline tools/evidence and an isolated smooth-contact model revision.

## Tasks

| ID | Task | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- |
| T01 | Freeze/reproduce Phase32 and Minimal authorities | Gate0 evidence | semantic replay + build/component | done |
| T02 | Freeze zeta/dzeta/manifold contract | `manifold-variable-contract.md` | frame/sign/nominal hash audit | done |
| T03 | Derive and validate affine ddzeta expression | coordinate oracle | independent FD/MuJoCo parity | done |
| T04 | Implement isolated Phase33 soft profile | source + `manifold-task-spec.md` | H/g-only and invariant tests | done |
| T05 | Run gain-free authority screen | `WBC-zeta-authority.md` | DG33-03 | done |
| T06 | Freeze and screen <=3 gain sets | gain evidence | DG33-04 | blocked |
| T07 | Retest C1 configuration closure | `C1-closure-retest.md` | DG33-05 | blocked |
| T08 | Retest C2 velocity closure | `C2-closure-retest.md` | DG33-05 | blocked |
| T09 | Audit manifold bandwidth | `manifold-bandwidth.md` | DG33-06 | blocked |
| T10 | Retest C3 spin separately | `C3-wheel-spin-retest.md` | DG33-07 | blocked |
| T11 | Build/audit smooth round-wheel diagnostic plant | `round-wheel-contact-audit.md` | matched contact gates | blocked |
| T12 | Re-run all closure families and decide minimum state | `minimum-state-decision.md` | DG33-08 | blocked |
| T13 | Run 20/40 ms then 100–400 ms rollouts | rollout reports | DG33-09 | blocked |
| T14 | Re-run original Phase29 T0/T1 | `T0-T1-corrective-recheck.md` | frozen derivatives | blocked |
| T15 | Regression/review and production decision | `REVIEW.md`; RECORD only on PASS | DG33-10 | blocked |

Task status is `todo / doing / done / blocked`.

## Failure Classes

`P33-A_wbc_normal_manifold_no_authority`, `P33-B_wbc_manifold_breaks_wrench_realization`,
`P33-C_C1_configuration_closure_not_recovered`, `P33-D_C2_velocity_closure_not_recovered`,
`P33-E_manifold_bandwidth_too_slow`, `P33-F_mesh_angle_is_contact_tessellation_artifact`,
`P33-G_wheel_spin_hidden_state_remains`, `P33-H_x16_closure_recovered`,
`P33-I_x18_minimum_candidate`, `P33-J_low_dimensional_closure_recovery_failed`,
`P33-K_closure_recovered_but_nmpc_still_noncorrective`, or `unresolved`.

## Validation and Evidence Rules

- Repository Python runs use `./.venv/bin/python`; dependency probe and `py_compile` precede stable
  output. Environment failures are not model evidence failures.
- Stable output paths refuse overwrite and contain command, interpreter/dependency versions, source,
  method, model and input hashes. Failed/inconclusive runs remain append-only.
- Save state, requested/algebraic-realized wrench, slack, torque, QP residuals, contacts, `zeta`,
  `dzeta`, `ddzeta`, pair identity and closure metrics.
- Production Phase27 remains frozen unless DG33-10 passes. A REVIEW with blocking findings is REWORK;
  RECORD is created only after PASS.

## Blockers

DG33-03: one frozen C1 state produced cross/self authority `0.5125767989`, above the predeclared
`0.5` isolation gate. Downstream gain, closure, bandwidth, contact and NMPC tasks remain blocked.
