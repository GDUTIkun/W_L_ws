# Phase 29 Root-Cause Audit Evidence

Authority run: `evidence/automated/phase29-root-cause-v1`  
Fresh replay: `evidence/automated/phase29-root-cause-v4`

## Authority and method

- Phase 28 input authority is `phase28-drift-attribution-v5`. T0 uses action
  tick `56` and snapshot tick `57`; T1 uses action/snapshot tick `44`.
- The production-lifecycle replay retains the acados iterate between 20 ms
  updates. The cold snapshot explicitly resets before a solve. The converged
  oracle is the namespaced, offline-only
  `phase29_wheel_aware_nmpc_sqp_v2` artifact and is never linked by production.
- Production prefix requested-wrench errors are `7.77e-16` for T0 and
  `7.22e-16` for T1, below the frozen `1e-8` gate. State/reference semantic
  errors are `3.47e-18` and exactly zero, below `1e-12`.
- Production and offline OCPs have the same 20-stage, 0.4 s discrete model,
  dimensions, `LINEAR_LS` costs, constraints, Gauss-Newton Hessian and partial
  condensing HPIPM QP solver. The declared differences are only model symbol
  names and offline `SQP`, convergence tolerances, iteration cap and
  globalization; production remains `SQP_RTI`.
- Frozen perturbations are `0.002 m/rad` for position/rotation and
  `0.01 m/s` or `rad/s` for rates. Every converged primary, perturbation and
  causal shadow solve satisfies the `1e-7` stationarity/feasibility gate. The
  saved stage trajectory, reference, cost, bounds, multipliers and residuals
  permit independent objective and KKT recomputation.

The earlier `phase29_wheel_aware_nmpc_sqp_v1` generated artifact is retained as
the superseded pre-formal diagnostic artifact. Replay-v2 is retained but
superseded because its manifest omitted explicit replay relations. Replay-v3
is a preserved environment-gate failure caused by a missing acados dynamic
library path; it contains no semantic outputs. Replay-v4 explicitly records
`replay_of=v1` and `supersedes=v2`.

## T0 static: P29-E horizon/terminal propagation

T0 remains non-restorative for production `+0.196866`, repeated RTI
`+0.196665`, converged SQP `+0.196665`, and cold solve `+0.197470`. Therefore
the effect is not a warm-start or one-RTI artifact (`P29-G`). State/reference,
exact-cost, model/control-authority and 100x bound-relaxation checks all pass.

Removing the full terminal objective changes the score to `-0.303705`.
Removing only terminal base-longitudinal terms changes it to `-0.295151`, while
removing terminal attitude, wheel-position, or wheel-rate terms leaves it
positive. Holding the reference across the horizon also leaves it positive.
Thus the isolated temporal mechanism is the terminal base-longitudinal
objective propagated through the finite-horizon coupled dynamics, classified
`P29-E_horizon_reference_propagation`. Single and pairwise state-group removals
are retained as interaction evidence and are not used as an order-dependent
primary classifier.

## T1 straight: P29-D cross-state coupling

T1 is non-restorative for production `+0.00113646`, repeated RTI
`+0.00114762`, converged SQP `+0.00114762`, and cold solve `+0.00107231`.
State/reference, objective, model authority and bound-causality gates pass;
held-reference, zero-terminal, terminal-group removals and 100x bound
relaxations all remain non-restorative, excluding `P29-E`, `P29-F`, and
`P29-G` as primary causes.

Removing the attitude group changes the score to `-0.00881819`; removing the
wheel-rate group changes it to `-0.000719806`. Pairwise results preserve the
attitude-dominant interaction. The acceleration decomposition independently
shows equilibrium `-0.0393364 m/s^2` opposed by common wheel force
`+0.0223573 m/s^2` and pitch moment `+0.00430066 m/s^2`, yielding the recorded
net non-restorative action. This closes T1 as `P29-D_cross_state_coupling`,
with attitude coupling primary and wheel-rate interaction secondary.

## T2 holdout

- Left turn is already restorative in production and converged solves
  (`-2.82e-4`, `-2.67e-4`), so it is `not_same` as T1.
- Right turn is non-restorative (`+2.97e-3`, `+2.99e-3`); attitude removal
  changes it to `-9.72e-3`, so it is `same` as the T1 dominant mechanism.

These holdouts do not assign a new T2 primary cause or approve a redesign.

## Replay and regression

- The five semantic JSON outputs in formal-v1 and replay-v4 are byte-identical.
  Their summary SHA-256 is
  `a86573b537bbbd783c1ff8c78ff7f64d41b531b2af0e2bbd7256870c5f9c2172`.
- Dependency probe: Python `3.10.20`, MuJoCo `3.7.0`, NumPy `2.2.6`, SciPy
  `1.15.3`, CasADi `3.7.2`; acados commit
  `21376cb1af6b7dd45f675367272d3ba8100b26c0`.
- Release `colcon build --packages-up-to wheel_leg_mujoco`: PASS.
- `colcon test` for `wheel_leg_core`, `wheel_leg_ros`, and `wheel_leg_mujoco`:
  `33 tests, 0 errors, 0 failures, 0 skipped`.
- Phase 28 directionality regression: PASS; derivatives remain T0
  `+118.153/+18.2632` and T1 `-0.972159/-0.491522`.
- A retry targeting formal-v1 exits before writing with `output already
  exists`, confirming non-overwrite behavior.

## Scope conclusion

The evidence approves attribution only. It does not modify or approve changes
to production cost weights, references, bounds, horizon, solver lifecycle,
WBC tasks, timing, public interfaces or fault/reset behavior.
