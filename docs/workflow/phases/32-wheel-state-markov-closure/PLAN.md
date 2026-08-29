# Phase 32: Wheel-State Markov Closure and Constrained-Dynamics Derivation — PLAN

Status: `review`

Design input: user-provided “Phase 32: Wheel-State Markov Closure and Constrained-Dynamics
Derivation”, approved with one contract clarification. The production-relevant input is the NMPC
requested internal wrench passed through Minimal WBC; requested and realized wrench are recorded
separately. A physical-wrench closure claim is allowed only when realized-wrench parity also passes.

## Goal

Decide whether the current 16-state projection is Markov-closed for wheel-relative acceleration. If
closed, derive a contact-regime-aware M4 reduction. If not, prove M5 with controlled
same-`x16`/same-request pairs and identify the minimum observable state augmentation.

## Frozen scope

- Reuse Phase31 T0/T1 authority, exact wheel-body-origin `xi/dxi`, MuJoCo 3.7.0 nominal plant,
  interaction-wrench frame/point/sign, Minimal WBC and Phase27 production artifact.
- No cost, terminal, reference, feedforward, horizon, solver lifecycle, WBC task, fast loop, safety,
  scalar inertia, polynomial or black-box change.
- Diagnostic-only, append-only outputs until closure and candidate gates pass. Rejected candidates do
  not remain in production source.
- C1/C2 start with local projection-rank/nullspace feasibility. A pair is required only for a real
  null direction; infeasible pairs are recorded, not fabricated.
- C3 prefreezes wheel-spin-rate common/differential pairs because wheel spin is present in full
  MuJoCo/RobotState but absent from `x16`, while it changes rolling/slip contact state.
- Amendment after C3: the current collision wheel is a faceted mesh. Wheel angle changes discrete
  contact-patch identity while leaving x16 unchanged. For this hybrid family the smooth full/half
  derivative gate is inapplicable; bilateral contact, independent oracle convergence, a per-pair
  `>0.05 m/s²` difference and byte-identical fresh replay are required instead.

## Grounding

- CBM project `W_L_ws`, generation `2026-08-29T06:47:42Z`, 9580 nodes/17210 edges, ready.
- Live chain remains `MuJoCo q/v → Adapter RobotState → NominalWbcModel closure reconstruction →
  xi/dxi → WheelAwareNmpcModel`.
- `RobotState` contains six active joint positions/rates, including wheel spin; the 16-state NMPC
  retains only base pose/twist and wheel-body-origin `xi/dxi`.
- Phase31 showed correct measurement semantics, valid plant `ddxi`, correct Eq.(12) signs, accurate
  WBC wrench realization, and mode-dependent gain failure inconsistent with scalar inertia.
- Existing Graphify history links Phase21 contact/Pfaffian oracles and Phase27 Eq.(12)/Minimal-WBC
  design. Historical results guide oracle reuse but do not override current MuJoCo truth.

## Frozen gates

- **DG32-00 Authority:** Phase31 semantic summaries and hashes reproduce; all six frozen facts pass.
- **DG32-01 Projection:** exact full→16 map, discarded variables and runtime observability documented.
- **DG32-02 Kinematics:** analytic/direct `xi,d​xi,ddxi` parity. Max `xi <=5e-5 m`, `dxi <=2e-3
  m/s`; acceleration oracle self-convergence `<=0.5 m/s²` inherited from Phase31.
- **DG32-03 Full oracle:** same q/v/controls/contact regime; numerical full `ddxi` must reproduce the
  independent MuJoCo oracle within `0.05 m/s²` on controlled pairs. Analytic reduction cannot proceed
  if this fails.
- **DG32-04 Pair identity:** per pair max reduced-state discrepancy `<=1e-9` for directly unchanged
  components and `<=1e-7` for reconstructed `xi/dxi`; requested wrench discrepancy `<=1e-12`.
  Both contacts and finite q/v/qacc are mandatory.
- **DG32-05 Closure:** acceleration scale `0.5 m/s²`; closure FAIL if any repeatable individual,
  common or differential difference exceeds `0.05 m/s²` (`normalized >0.1`). The pair must pass
  full/half perturbation consistency within `10%` and fresh replay.
- **DG32-06 Wrench separation:** record requested and realized wrench. Realized mismatch is a separate
  composed-controller mechanism; physical-wrench closure requires normalized realized discrepancy
  `<=2%`.
- **DG32-07 Decision:** choose P32-B/M4 or P32-C plus P32-D/E/F/M5. Multiple causal hidden families
  remain `unresolved`; do not select the easiest repair.
- **DG32-08 Candidate sensitivity:** only after decision; all four primary gains within `20%`, cross
  ratio `<=10%`, and materially better than Eq.(12).
- **DG32-09 Rollout:** only after sensitivity; 20/40 ms first, wheel-rate normalized error `<0.1`
  (target `<=0.05`), then 100/200/400 ms with no base-group regression.
- **DG32-10 Return/release:** original Phase27 formulation and RTI/SQP only after rollout; production
  integration requires all local, closed-loop, fault, deadline and regression gates.

## Tasks

| ID | Task | Deliverable | Status |
| --- | --- | --- | --- |
| T01 | Freeze Phase31 authority and reproduce | authority method/evidence | done |
| T02 | Define full-state→16-state projection and discarded variables | `full-to-reduced-state-contract.md` | done |
| T03 | Freeze exact wheel-coordinate acceleration kinematics | `wheel-coordinate-kinematics.md` | done |
| T04 | Build full numerical MuJoCo `ddxi` oracle | `full-ddxi-oracle.md` | done |
| T05 | Freeze pair method, scales and C1–C3 perturbations | `markov-closure-method.md` | done |
| T06 | Audit C1 leg-configuration nullspace/pairs | closure evidence | done |
| T07 | Audit C2 leg-velocity nullspace/pairs | closure evidence | done |
| T08 | Run C3 wheel-spin/contact-regime pairs | closure evidence | done |
| T09 | Freeze M4/M5 and hidden-state attribution | decision documents | done |
| T10 | M4 derivation or M5 minimum augmentation | branch-specific specification | done |
| T11 | Candidate sensitivity and independent parity | validation evidence | blocked |
| T12 | 20/40 then 100–400 ms rollout | rollout reports | blocked |
| T13 | Original T0/T1 corrective recheck | corrective report | blocked |
| T14 | Production/regression review | REVIEW; RECORD only on PASS | blocked |

Task status is `todo / doing / done / blocked`.

## Failure classes

`P32-A_full_dynamics_oracle_incomplete`, `P32-B_16state_markov_closure_confirmed`,
`P32-C_16state_markov_closure_failure`, `P32-D_leg_configuration_hidden_state`,
`P32-E_leg_velocity_hidden_state`, `P32-F_contact_regime_hidden_state`,
`P32-G_reduced_model_candidate_sensitivity_failure`, `P32-H_rollout_failure_after_local_match`,
`P32-I_wheel_model_repaired_but_nmpc_still_noncorrective`, or `unresolved`.

## Evidence rules

- Use `./.venv/bin/python`; probe MuJoCo/NumPy/SciPy and `py_compile` before stable outputs.
- Save qpos/qvel/qacc, contact IDs/dimensions, penetration, contact-frame slip, normal/tangent force,
  WBC torque, requested/realized wrench, projection residuals, perturbation and input/source hashes.
- Safety-latched or `dt<=0` samples are invalid. Output directories refuse overwrite; invalid and
  inconclusive runs remain append-only.
- Correlation/regression may screen candidates only. Closure decisions require controlled pairs and
  fresh semantic replay.
