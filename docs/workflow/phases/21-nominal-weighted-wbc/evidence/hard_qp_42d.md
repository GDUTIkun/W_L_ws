# Phase 21 42D Hard-QP Freeze and Oracle

Date: 2026-08-27  
Scope: P21-T04 only; no weighted-task, nonlinear-holdout, Core, ROS, hardware, or NMPC authority

## Frozen physical variable contract

The physical decision vector has 42 entries:

```text
z = [nudot_12, tau_6, w_left_C_6, w_right_C_6,
     slack_left_FLU_6, slack_right_FLU_6]
```

- `nudot_12` follows the Phase-21 tangent: world-axis base linear acceleration,
  world-axis base angular acceleration, then canonical
  `[left hip, left knee, left wheel, right hip, right knee, right wheel]`
  active-joint accelerations.
- `tau_6` has the same canonical joint order and N·m sign as `TorqueCommand`.
- Each `w_side_C` is the wrench about the analytic continuous contact center,
  expressed in the frozen contact frame and ordered
  `[rolling, lateral, normal, rolling-moment, lateral-moment, normal-moment]`.
  The left block precedes the right block.
- Each future fidelity slack is a controller-FLU wrench ordered
  `[Fx,Fy,Fz,Tx,Ty,Tz]`. Its sign remains
  `W_feasible = W_reference + slack`. Slack appears in no hard row and is fixed
  only by the unit minimum-scaled-norm tie-breaker in this audit. P21-T06 must
  define the frame transform and weight before using it as a soft task.

The old 36D single-force variable and its 24 equality rows are historical only.

## Frozen hard equations and row map

The physical reduced dynamics are

```text
M_r nudot + h_r = S_r tau + B_left w_left_C + B_right w_right_C.
```

The solver receives scaled coordinates `z = D x` and bound form
`l <= A x <= u`. There are exactly 104 rows, derived as
`12 + 6 + 2*37 + 12`:

| Rows | Count | Physical meaning |
| --- | ---: | --- |
| 0–11 | 12 | reduced dynamics equality |
| 12–17 | 6 | canonical torque box |
| 18–54 | 37 | left fixed contact-centred H-cone, `H_C w_left_C <= 0` |
| 55–91 | 37 | right fixed contact-centred H-cone, `H_C w_right_C <= 0` |
| 92–103 | 12 | componentwise acceleration box |

The exact 37-by-6 physical `H_C` coefficient matrix is emitted in every
`summary.json` as `contact_cone_H_physical`. It is regenerated from the already
frozen 24-ray, six-point, `mu=1` cone and checked to contain 37 rows. The runtime
row count must be derived from the row blocks; it must not retain the old 36D
equality count or another magic total.

## Protection audit

The retained acceleration box is
`[10,10,10,20,20,20,50,50,50,50,50,50]` in the physical `nudot` order. It is a
componentwise numerical/safety envelope, not a joint-position barrier.

No state-dependent joint position/velocity protection, look-ahead horizon, or
joint-limit profile has been frozen anywhere in the current Phase authority.
P21-T04 therefore does not invent one or copy the Simulink implementation.
Position-aware joint protection remains an explicit P21-T05 decision gate.

## Scaling and objective used for the solver audit

Variable scales are acceleration limits, torque limits, and per-side wrench/slack
scales `[50,50,50,2.5,2.5,2.5]`. Dynamics rows use the existing physical scales
`[100,100,100,20,20,20,10,10,10,10,10,10]`. Each cone row is divided by the
2-norm of `H_i D_w` after variable scaling; this leaves the physical cone exactly
unchanged while making every solver cone row unit norm. Torque and acceleration
rows are normalized to bounds `[-1,1]`.

The audit objective is `0.5 ||x||^2`. It is only a deterministic,
strongly-convex minimum-scaled-norm tie-breaker for feasibility and solver
comparison. It is not a standing, contact-acceleration, wrench-fidelity, or task
weight freeze.

## Independent corpus and acceptance

`validate_weighted_wbc_hard_qp_42d.py` rebuilds `M_r/h_r/S_r/B_left/B_right`
from the Phase-21 model and contact oracles, then audits four workspace states and
28 selected rolling ticks, including the previously blocking 212–220 region.
SciPy HiGHS independently establishes feasibility; SLSQP with analytic convex
objective and linear Jacobians supplies a QP candidate. The validator recomputes
hard residual, active-set stationarity, complementarity, cone row norms, and
scaled normal-matrix conditioning. A contradictory-equality case must be
infeasible; non-finite, indefinite, inconsistent-bound, and iteration-limit
failure semantics are exercised by the C++ component tests.

DG21-03 can close only if the 42D C++ dense ADMM agrees with this corpus and its
1000-run cold, repeated-same warm, and dynamic-warm benchmark passes. This oracle
alone does not close DG21-04 or authorize P21-T05/P21-T06.
