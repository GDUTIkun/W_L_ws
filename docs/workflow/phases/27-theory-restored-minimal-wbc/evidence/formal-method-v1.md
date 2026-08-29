# Phase 27 Formal Method v1

Date frozen: 2026-08-29, before creation of the primary formal output root.

Decision: `DG27-05 PASS`

## Matrix and references

The immutable machine-readable authority is
`simulation/mujoco/config/phase27_minimal_formal_v1.json`.

- T0: 6 s static hold from the current nominal equilibrium.
- T1: 6 s straight case. The longitudinal reference ramps linearly from 0 to
  0.20 m/s in 1 s, cruises through 3 s, brakes linearly to zero by 4 s and
  remains stopped for 2 s.
- T2: separate 6 s left/right cases. Longitudinal speed and yaw rate ramp
  linearly for 1 s to 0.20 m/s and +/-0.08 rad/s, then remain constant. The
  yaw-aligned chart anchor is refreshed at each NMPC solve.
- T3: separate 4 s static-reference cases with independently reconstructed
  closed-chain initial equilibria giving `xi_delta(0)=+/-0.010000 m` while
  preserving each wheel's nominal vertical position. No differential WBC task
  is enabled.

There is one deterministic primary episode per case. Fresh-process replay is
separate T11 evidence. There is no tuning matrix: OCP weights/bounds were
closed by T06 v2 and Minimal WBC weights by T07 before this method was frozen.

## Gate ordering

Hard gates are evaluated first: row completion/finite values, Core/Adapter,
NMPC and WBC status, 2/10/20 schedule and age, 10 ms combined deadline,
independent NMPC defect/stationarity/bounds, QP hard feasibility, exact torque
limits and ZOH, bilateral contact/normal load, and the existing x/y/z/attitude
safety envelopes. A hard failure makes the case FAIL regardless of tracking.

Performance is still recorded after a hard failure for attribution, never to
override it. It includes velocity/yaw-rate tracking, wheel common/differential
state, requested/realized interaction wrench and signed slack, wrench rate,
torque/contact resource, and T3 tail recovery. Thresholds are exactly those in
the frozen JSON; they are not revised after the primary run.

## First-failure attribution

The evaluator emits one of the frozen enums in the JSON and stores the first
failing control row plus the corresponding plant rows. Classification order is
nonfinite, contact/plant, deadline, NMPC, interface, WBC, safety envelope, then
planner. This order uses direct diagnostics and does not infer success from a
process exit code. A Minimal FAIL with healthy upstream component gates is an
allowed Phase 27 terminal result and does not authorize an add-back task.

The evaluator has a synthetic self-test which constructs one PASS trace and
one trace for each failure enum, including threshold equality and first-row
selection. The self-test and `py_compile` must pass before the stable formal
output directory is created.
