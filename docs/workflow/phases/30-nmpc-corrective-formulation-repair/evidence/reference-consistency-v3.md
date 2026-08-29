# Phase 30 v3 Reference-Consistency Evidence

Date: 2026-08-29  
Outcome: `REWORK — P31-F_wheel_state_model_adequacy_failure`

## Frozen Method

- Gate 0–2 primary method is byte-preserved as
  `simulation/mujoco/config/phase30_reference_consistency_gate12_v1.json`; v3 adds only the
  pre-frozen conditional Branch-M thresholds.
- Exact map: Phase27 generator `disc_dyn_expr`, 20 ms with two 10 ms RK4 substeps.
- Samples: T0 updates 54/56/58 and T1 updates 42/44/46, all 20 horizon stages.
- Defect: `x_ref[k+1] - f_d(x_ref[k], u_ref[k], R_ref)`, normalized by the frozen
  Phase27 state scales.
- Case C gate: current maximum normalized defect at or below `1e-3`.
- Branch-M 20 ms realized-input model gate: maximum normalized prediction error at or below `0.1`.
- No Q/Qe, terminal, horizon, solver, bounds, WBC, lifecycle or production changes were permitted.

## Gate 0 — Authority

| Case | Semantic max | Prefix/request max | Production u0 | Converged u0 | Result |
| --- | ---: | ---: | ---: | ---: | --- |
| T0 | 0 | 7.77e-16 | 0 | 0 | PASS |
| T1 | 0 | 7.22e-16 | 0 | 0 | PASS |

## Gates 1–2 — Reference Feasibility

All six update problems gave the same current maximum normalized defect,
`8.737811676854253e-4`. Bounded best-input minimization terminated successfully for every stage and
reduced the maximum to approximately `6.65002049e-5`. Because the current reference is already below
the pre-frozen small gate, both cases are `P31-C_current_reference_already_consistent`. The reduction
under best input is diagnostic only and does not authorize a stage-feedforward change.

## Branch M — Recorded-MuJoCo Predictor

| Case ticks | 20 ms realized-input normalized max | 40 ms | 100 ms |
| --- | --- | --- | --- |
| T0 54/56/58 | 0.2026 / 0.2065 / 0.2258 | 0.2247 / 0.3148 / 0.4145 | 0.7905 / 1.0090 / 1.0220 |
| T1 42/44/46 | 0.1296 / 0.1598 / 0.1960 | 0.2894 / 0.3455 / 0.2684 | 0.5627 / 0.8292 / 0.7716 |

At 20 ms the largest individual errors are wheel-center relative rates: `0.0194..0.0340 m/s`, or
`0.1296..0.2258` of their frozen scale. Requested-input and realized-input rollouts are nearly
identical, while base attitude/velocity groups remain substantially smaller. The evidence therefore
supports a wheel-state kinematics/dynamics/measurement-contract mismatch, not WBC wrench realization
and not a blanket rejection of the base rigid-body equations.

The original diagnostic continuation latches at T0 tick 70 and T1 tick 62. Consequently 200/400 ms
recorded comparisons are unavailable and are explicitly omitted rather than treating latched `dt=0`
rows as plant motion. Since every neighborhood already fails the 20 ms local gate, longer rollout is
not required to reject the current 16-state model as production repair authority.

## Replay and Artifacts

- `evidence/automated/phase30-reference-consistency-v1` and fresh `v2` have byte-identical
  `reference_defect_audit.json` and `summary.json`.
- The v1 manifest method hash matches the preserved Gate 0–2 method; v2 uses the append-only v3
  method whose only added fields belong to Branch M. Gate 0–2 semantic outputs remain identical.
- `evidence/automated/phase30-model-adequacy-v1` and fresh `v2` have byte-identical
  `model_adequacy.json` and `summary.json`.
- The first reference run without the acados library path failed before creating its output directory;
  it is an environment event, not control evidence.
- Production solver, cost, reference, WBC and controller code remain unchanged.

## Approved Next Direction

The next repair must first close the meaning and dynamics of `xi_left/right` and `dxi_left/right`
against MuJoCo wheel-center kinematics. It should compare measured finite-difference `dxi`, Adapter
reported `dxi`, and the Eq.(12) wheel acceleration under common/differential `Fx/Ty` probes. Only after
20/40/100/200/400 ms model-to-plant gates pass may a reference-consistent 12D stage feedforward be
retested with the existing cost and solver.
