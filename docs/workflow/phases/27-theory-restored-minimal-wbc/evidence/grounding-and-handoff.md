# Phase 27 grounding and Phase 26 handoff

Date: 2026-08-29

Decision: `DG27-00 PASS`

## Authority and coverage

- Live-code authority is CBM project `W_L_ws`, generation
  `2026-08-28T13:13:14Z`, followed by direct reads of every touched or
  metadata-changed source file. The queried Core/WBC paths had no recorded
  indexing gap; newer worktree files were nevertheless read directly.
- Historical-design authority is the existing local Graphify graph. The
  Phase 21--23 path confirms the approved progression from reconstructed
  nominal model, through 42-variable Weighted WBC, to append-only Phase 23
  12-state NMPC. Current source wins wherever history and worktree differ.
- `docs/` and `tools/` are intentionally outside the CBM source index and
  were read directly.

## Phase 26 handoff

At the user-directed handoff Phase 26 contained only its PLAN. `P26-T01`--
`P26-T10` were all `todo`; there was no source, config, log, evaluator, formal
run, REVIEW or RECORD to inherit. Phase 26 remains a blocked current-12D
audit. Its task-necessity conclusions and thresholds are not evidence for the
new physical state, OCP, wrench interface or candidate schedule.

## Reuse boundary

| Asset | Decision | Phase 27 use |
| --- | --- | --- |
| Canonical `RobotState -> TorqueCommand` | reuse unchanged | Public boundary and joint/frame conventions |
| `NominalWbcModel` reconstructed 12-DoF model | extend additively | Wheel state and Newton--Euler maps at fixed `q,nu` |
| Phase 21/22 42D order, 104 hard rows and ProxQP | reuse unchanged | Minimal profile changes objectives only |
| Phase 23 canonical base state and MuJoCo wheel geometry oracle | reuse | Independent `xi_L/R,dxi_L/R` authority |
| Phase 23 12-state model/solver/generated artifact | do not alter | Default-mode regression baseline only |
| Phase 23 external base-control-point resultant wrench | do not reinterpret | It is not internal wheel interaction wrench |
| Phase 23 2/10/20 ms schedule | comparison baseline | No approval until DG27-04 |
| Historical Simulink governor and Eq. (12) | oracle input only | Re-derived in current frames and parameters |

## Ownership and impact

Phase 27 code, generated artifacts, configs, scripts, schemas, modes and
formal roots use a `phase27` or `wheel_aware` namespace. Existing Phase 21--23
goldens, generated C, configs and evidence remain immutable.

Implementation impact is
`NominalWbcModel wheel geometry/dynamics`
→ `WheelPositionPlanner + Nmpc16 model/solver`
→ `Minimal interaction-wrench objective`
→ `ControllerCore opt-in scheduler`
→ `wheel_leg_mujoco runner/log/evaluator`.
The Phase 23 path remains a parallel baseline, not an in-place migration.
