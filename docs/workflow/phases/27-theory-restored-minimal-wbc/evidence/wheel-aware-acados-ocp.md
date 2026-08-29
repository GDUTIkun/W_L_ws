# Wheel-aware acados OCP and solver

Date: 2026-08-29

Decision: `T06 PASS`

## Frozen OCP contract

The OCP uses the approved 16-state model and wheel-on-body internal wrench
input without changing actor, point, frame or order. It retains SQP-RTI with
partial-condensing HPIPM, a 20 ms sample, 20 stages and a 0.4 s horizon. The
discrete model contains the same two fixed 10 ms RK4 substeps as the C++ model.

At every solve, the 9 parameters hold one fixed `R_ref`. Stage references and
state-envelope centers advance from the initial reference with the frozen
`v`, `omega` and per-side `dxi`; this makes nonzero velocity and yaw-rate
references kinematically consistent. Wheel bounds are intersected with the
physical per-side workspace at every predicted stage. Input bounds apply to
the internal wrench at each wheel center. No input-rate state or hidden task is
added; wrench-rate formal thresholds remain owned by DG27-05/T09.

The v2 running input normalization reuses the already validated Phase 23
scale hierarchy while changing the equilibrium and wrench meaning to the new
internal contract. State cost adds symmetric per-side `xi/dxi` terms. These
are component-profile values, not controller-level performance evidence.

## Generation and model parity

The generator refuses nonempty output, pins acados commit, CasADi version and
the Tera renderer hash, and is not invoked by CMake. Two clean v2 generations
and the checked-in artifact have byte-identical generated C/H/PXD sources.
After replacing only the absolute output path, all three Makefiles hash to
`705245a7...2309`; normalized `acados_ocp.json` files hash to
`7bf91c8b...24f15` after excluding its path-derived hash field.

CasADi generated dynamics were compared with the independent six-sample v2
model oracle. Maximum next-state, state-Jacobian and input-Jacobian errors are
`1.11e-16`, `1.038e-10` and `1.038e-10`, respectively.

## Solver and audit results

The C++ wrapper checks finite/SO(3)/chart/workspace inputs, applies a
kinematically advancing horizon reference, audits all predicted states and
inputs, independently recomputes full-horizon dynamics defects and projected
stationarity, and provides deterministic cold reset. The independent running
cost audit includes the acados `EULER` discretization factor `Ts`; omitting it
was the defect in the initial v1 audit.

Equilibrium, positive/negative reference, brake, return, wheel-common and
wheel-differential cases all PASS. A 300-solve dynamic warm corpus reports
`p99=3.833 ms`, `max=3.958 ms`, maximum full-horizon defect `1.633e-4` and
maximum projected stationarity `5.954e-4`, against frozen gates of `10 ms`,
`1e-3` and `0.05`. Workspace and non-finite inputs reject; repeated cold
reset is bit-deterministic. The Release build through `wheel_leg_mujoco` and
the Core/ROS/MuJoCo suite pass with 30 tests and no errors or failures.

The v1 artifact is retained but withdrawn as inconclusive: its apparent
projected-stationarity failure used the invalid unscaled audit. No threshold
was widened, and no v1 controller claim is made.
