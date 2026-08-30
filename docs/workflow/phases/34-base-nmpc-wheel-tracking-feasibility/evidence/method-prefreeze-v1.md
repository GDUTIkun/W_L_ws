# Phase 34 method prefreeze v1

Date: 2026-08-30

Decision: T02 freezes the execution order and numerical gates before any Phase34 candidate result is
read. The machine-readable authority is
`simulation/mujoco/config/phase34_feasibility_v1.json`.

Frozen SHA-256:

- Phase34 method: `167b4ad4e4cb888e3909698a03fb84bf1ff27704ff8e6a6c0ba74d4df75f908f`
- Phase34 OCP: `ce46051da820c07e5b75d4d379c2eee25d1d1258e3544b66d0d86ca9fee4b110`
- Phase27 OCP authority: `c24159aebbd7b38380044319e9cfdb619d880b86f9050aedc086ab07aba5eadf`
- Phase29 method authority: `557d8fa63ec39bca53cee5eadfc58f6c6feb156dab1ae95be7541da32788dcfa`

## Frozen order

`model -> generated OCP -> longitudinal WBC affine map -> gain-free authority -> <=3 gains ->
T0/T1 corrective -> static -> bounded straight -> bounded turning -> regression`.

The first blocking failure stops downstream execution. A failure does not authorize slack/workspace
feedback, state augmentation, Q/Qe/R/terminal changes, a differential planner or Eq.(12).

## Model and solver gates

- The independent one-step reference is DOP853 at `rtol=1e-12`, `atol=1e-14`; the candidate remains
  two fixed 10 ms RK4 substeps.
- Maximum one-step, state-Jacobian, input-Jacobian and `xi`-parameter-Jacobian errors are respectively
  `3e-8`, `2e-6`, `2e-6` and `2e-6`.
- CasADi/generated-expression parity is `1e-12` for next state and `2e-9` for Jacobians.
- The solver keeps the Phase27 `10 ms` deadline, `1e-3` dynamics-defect and `0.05` projected-
  stationarity gates. The converged SQP oracle uses `1e-7` stationarity/feasibility gates.

## WBC gates

Before gains, centered `1 m/s^2` common and differential acceleration requests must produce signed
self gain `>=0.2`, cross/self `<=0.5`, 2x2 condition number `<=10`, realized-wrench relative change
`<=2%` and hard violation `<=1e-7`. The requested wrench is fixed. At most three gain sets may be
chosen only after this gate, and a `5 mm` common step / `0.02 m/s` ramp must finish within `1 mm`,
keep differential drift within `2 mm`, and settle within `0.2 s`.

## Corrective and closed-loop gates

Phase29 T0/T1 state/reference semantics and action ticks are reused. A corrective score is
`(state_error on the authority rate) * predicted acceleration`; it must be below `-1e-7` for both
RTI lifecycle and converged SQP. Static and motion gates reuse Phase27/28 metric definitions and may
not relax the current `0.20 m/s` / `0.08 rad/s` command contract.

## Evidence

Stable outputs refuse overwrite and record interpreter/dependencies, source/config/generated/input
hashes, command, seeds, status and replay/supersedes relationships. Environment failures are not
model/control FAIL.

Dependency probe: repository `./.venv/bin/python`; NumPy `2.2.6`, SciPy `1.15.3`, MuJoCo `3.7.0`,
CasADi `3.7.2`; JSON parsing and repository diff check PASS.
