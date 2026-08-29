# Phase 28 Review

Outcome: `PASS`

Reviewed authority: `phase28-drift-attribution-v5`; fresh replay:
`phase28-drift-attribution-v6`.

## Findings

Blocking findings: **0**.

- T0 and T1 reproduce the Phase 27 nominal failures at ticks `58` and `45` with
  zero semantic prefix difference. Their diagnostic continuations stop at ticks
  `70` and `62`; neither recovers.
- Both primary cases uniquely stop at decision-tree layer B. T0 has reinforcing
  pitch acceleration and positive frozen-state pitch/pitch-rate response
  derivatives. T1 has locally negative perturbation derivatives, but its
  snapshot and first-divergence window produce net longitudinal acceleration in
  the same direction as the negative position/velocity error. Thus the executed
  NMPC action is not restorative in the relevant window.
- Layer C is excluded: maximum force residual is `0.001403 N`, maximum moment
  residual is `1.2601e-5 Nm`, maximum hard violation is `7.33e-10`, minimum
  wheel normal load is `30.58 N`, and minimum torque margin is `1.99 Nm` across
  T0/T1 attribution windows.
- Layer D is excluded for the primary cases. Direct MuJoCo acceleration agrees
  with the centered velocity finite difference to at most `7.19e-5 m/s²`
  linear and `4.67e-7 rad/s²` angular RMS. NMPC-to-WBC and WBC-to-plant gates
  are both PASS.
- T2 is deliberately not assigned a primary mechanism. Right turn follows the
  T1 B path; left turn does not. The Phase result is therefore
  `not_consistent`, not a new E-layer or task-necessity claim.

## Invalid and superseded runs

- `v1` remains an alignment FAIL. Its evaluator compared a centered 20 ms
  finite difference against only one 10 ms acceleration interval.
- `v2` fixed that timing alignment without changing thresholds. `v3` replayed
  it. `v4` added resource reporting and ambiguity handling. `v5` clarified that
  T2 has no primary attribution; `v6` is its fresh replay. No prior directory
  was overwritten.

## Verification

- Dependency probe: MuJoCo `3.7.0`, NumPy `2.2.6`, SciPy `1.15.3`.
- Python compile and evaluator synthetic classification, including ambiguous
  input to `unresolved`: PASS.
- Release `colcon build --packages-up-to wheel_leg_mujoco`: PASS.
- `colcon test` for `wheel_leg_core`, `wheel_leg_ros`, and
  `wheel_leg_mujoco`: `33 tests, 0 errors, 0 failures, 0 skipped`.
- Frozen-state NMPC directionality oracle: PASS with T0 derivatives
  `+118.153/+18.2632` and T1 derivatives `-0.972159/-0.491522`.
- `v5` versus `v6`: 480 semantic control rows with zero differences, exact
  plant files, exact summary.
- Phase 27 normal formal regression: semantic summary equals formal-v2 after
  excluding declared timing metrics; expected conclusion remains
  `Minimal FAIL` with unchanged failure layers.
- Non-overwrite retry: rejected before writing.

## Scope review

The change is simulation-only and opt-in. It adds MuJoCo acceleration logging,
an expanded diagnostic pose envelope, an offline evaluator, and one component
oracle. It does not change the Phase 27 normal control law, public command/state
interfaces, OCP/WBC topology, task set, solver, timing, fault priority, or
formal thresholds. No compensation task or tuning recommendation is approved.

