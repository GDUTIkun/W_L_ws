# Phase 31: Wheel-State Model and Measurement Contract Audit — PLAN

Status: `review`

Design input: user-provided “Phase 31: Wheel-State Model and Measurement Contract Audit”. The route is
approved with one live-code correction: `Adapter` does not publish `xi/dxi`. The actual chain is
`MuJoCo q/base state → Adapter RobotState → NominalWbcModel closed-chain reconstruction → xi/dxi →
16-state NMPC`. All gates below use that chain.

## Goal

Identify why the frozen 16-state model misses MuJoCo `dxi_left/right` after 20 ms, distinguish
measurement semantics from Eq.(12), parameter and omitted-coupling failures, and form the smallest
wheel-channel candidate only after a unique root cause is supported.

## Scope and Frozen Boundaries

- Preserve Phase30 T0/T1 authority, 16-state ordering, interaction-wrench point/frame/sign, current
  nominal MuJoCo plant and Phase27 timing.
- Audit wheel-body origin relative to `base_control_frame`; this is not wheel rolling angle or contact
  patch arc length. Positive `xi` is body/FLU +x.
- Compare core-reconstructed `xi/dxi` with direct MuJoCo body/site geometry, analytic relative
  velocity and valid centered finite differences.
- Only after measurement PASS, audit plant `ddxi`, Eq.(12), common/differential input sensitivities,
  residual coupling and effective inertia.
- Only after root-cause closure may a candidate alter wheel-state measurement/dynamics. Cost,
  reference, input reference, horizon, solver lifecycle, WBC tasks and production safety remain frozen.
- No arbitrary correction factor. A scalar effective inertia is admissible only if sign/cross-coupling
  gates pass and the gain is stable across the frozen state corpus.

## Current Grounding

- CBM project `W_L_ws`, generation `2026-08-29T06:47:42Z`, 9580 nodes/17210 edges, ready.
- `Adapter::extractState` exports base-control-site pose/twist plus six canonical joint positions and
  velocities; no wheel reduced coordinate exists in `RobotState`.
- `ControllerCore::stepPhase27MinimalNmpcWbc` obtains states 12–15 from
  `NominalWbcModel::evaluate`.
- `NominalWbcModel::evaluate` reconstructs four passive closed-chain joints, then defines each `xi`
  as the wheel-body origin relative to the base-control site, expressed in base FLU, x component. It
  defines `dxi` as the derivative in the rotating base frame, including `-omega_b × r_b`.
- Phase30 Gate 0–2 authority remains valid. Its Branch-M recorded predictor is the Gate0 mismatch
  authority, not proof that Eq.(12) is wrong.
- `docs/` and `tools/` are deliberately outside CBM. `weighted_wbc_loop.cpp` changed after the indexed
  generation and is read directly. `adapter.hpp:28` has a recorded parse-partial range and was read.

## Frozen Gates

- **DG31-00 Authority:** exact source/config/run hashes and Phase30 20 ms error reproduction.
- **DG31-01 Position semantics:** direct MuJoCo wheel-body-origin coordinate vs core log; max absolute
  error `<=5e-5 m`. Sign comparison ignores magnitudes below `1e-6 m`.
- **DG31-02 Velocity semantics:** analytic MuJoCo relative velocity vs core log and centered 2 ms
  geometry FD; each max absolute discrepancy `<=2e-3 m/s`. `dt<=0`/latched samples are invalid.
- **DG31-03 Acceleration oracle:** centered velocity FD and, where available, analytic kinematic
  acceleration must agree within a pre-frozen tolerance before Eq.(12) attribution.
- **DG31-04 Eq.(12):** same state/frame/wrench point; save requested/realized and common/differential
  residuals. Requested/realized agreement cannot by itself prove model correctness.
- **DG31-05 Input response:** common/differential `Fx/Ty` signs, gains, cross-coupling and left/right
  symmetry from same-state controlled perturbations.
- **DG31-06 Root cause:** choose only M1 measurement, M2 sign/frame, M3 constant parameter, M4 omitted
  coupling or M5 unsuitable reduced coordinate; multiple supported classes remain unresolved.
- **DG31-07 Candidate:** only the proven minimum measurement/dynamics change; independent/CasADi/C++
  parity required.
- **DG31-08 Rollout:** wheel-rate normalized error `<0.1` at 20 ms, preferred target `<=0.05`, then
  frozen-input 40/100/200/400 ms; base-state groups may not regress.
- **DG31-09 NMPC return:** original Phase27 cost/reference/input reference/horizon/SQP-RTI only; rerun
  frozen T0/T1 local gates before any production integration.
- **DG31-10 Release:** static artifact, closed loop, T2/T3, fault/deadline/history and colcon only after
  all prior gates PASS.

## Tasks

| ID | Task | Deliverable | Status |
| --- | --- | --- | --- |
| T01 | Freeze authority, live chain, IDs, hashes and method | grounding/contract/config | done |
| T02 | Reproduce Phase30 20 ms mismatch | baseline evidence | done |
| T03 | Freeze exact `xi/dxi` semantics | `wheel-state-contract.md` | done |
| T04 | Position parity corpus | raw/grouped evidence or P31-A | done |
| T05 | Velocity adapter/analytic/FD parity | raw/grouped evidence or P31-B | done |
| T06 | Independent plant `ddxi` oracle | acceleration evidence | done |
| T07 | Eq.(12) direct comparison | requested/realized residual report | done |
| T08 | Same-state common/differential `Fx/Ty` perturbations | input-response matrix | done |
| T09 | Residual coupling/effective-inertia audit | controlled attribution | done |
| T10 | Freeze root-cause class | M1–M5 decision | done |
| T11 | Implement minimum candidate and parity tests | candidate specification/code | blocked |
| T12 | 20–400 ms rollout | one/multi-step reports | blocked |
| T13 | Recheck original T0/T1 formulation | corrective matrix | blocked |
| T14 | Production/regression/review | REVIEW; RECORD only on PASS | blocked |

Task status is `todo / doing / done / blocked`.

## Failure Classification

`P31-A_wheel_position_semantics_mismatch`, `P31-B_wheel_rate_measurement_semantics_mismatch`,
`P31-C_configuration_dependent_effective_inertia`, `P31-D_eq12_sign_or_frame_error`,
`P31-E_missing_wheel_kinematic_dynamic_coupling`, `P31-F_reduced_wheel_coordinate_inadequate`,
`P31-G_wheel_model_repaired_but_nmpc_still_noncorrective`, or `unresolved`.

## Validation and Evidence Rules

- Use `./.venv/bin/python`; dependency probe and `py_compile` precede stable output creation.
- Every direct MuJoCo sample stores qpos/qvel, body/site IDs, time, safety/latch status, raw coordinate,
  common/differential transforms and input/config/source hashes.
- Use full 2 ms plant rows; a control tick at time `t` corresponds to the previous control tick's final
  physics row at `t`. Alignment must be independently checked against the control-row base pose/twist.
- Invalid/environment/inconclusive outputs are append-only. Fresh replay semantic outputs must be
  byte-identical. Existing output directories are refused before work.
- Production remains Phase27 until DG31-00–10 all PASS. REVIEW with any blocking finding is REWORK;
  no RECORD is created before PASS.
