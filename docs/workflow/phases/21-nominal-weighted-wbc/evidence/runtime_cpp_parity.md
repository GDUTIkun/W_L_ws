# P21-T07 Runtime C++ Model, Problem and Solver Parity

Date: 2026-08-28
Verdict: **PASS — P21-T07 complete; Controller Core integration remains P21-T08**

## Runtime boundary

`wheel_leg_core` now contains an Eigen-only analytic nominal model, fixed 42D/104-row
problem assembler and thin solver wrapper. These components do not include or link MuJoCo,
do not read plant truth and do not modify public `RobotState` or `TorqueCommand`.

The analytic model reconstructs the four passive joints on the frozen branch and evaluates
the 16D tree to 12D reduction, mass, bias, actuation, contact Jacobian/bias, generalized
wrench map and contact-wrench-to-controller-FLU map. The reconstruction solve uses a
`1e-12 m` internal stop, stricter than the unchanged `1e-10 m` acceptance gate.

## Golden parity

Both fresh golden exporters reproduce byte-identically on a second output path.

- model golden v2 SHA-256: `66e19399a54ffbfd0e1adb709a58360322767750710a1bf97a7269d8583919f3`
- problem golden v2 SHA-256: `0c49680ff833be3b771b8eb51d66e124f268e295953d307a9168e05359e29a39`

The model test covers four workspace states, dynamic ticks 68/204/259 and the explicit
tick-271 rejection. At the tightest in-workspace tick 259, maximum contact-bias error is
`6.27944e-8 m/s²`; all other reported model blocks remain within their frozen tolerances.
Across all 32 workspace-aware problem cases, maximum matrix/vector differences are below
`9.46e-13` for `H`, `1.215e-8` for `g`, `1.40e-14` for `A`, and `1.54e-14` for `l/u`.

## Solver audit

Weighted task Hessians exposed an unaudited limitation of the original unrelaxed ADMM:
several cases reached 10,000 iterations. The solver now has validated standard
over-relaxation `alpha=1.6` (default) and the weighted wrapper uses `rho=0.15`; hard-QP
behavior was rerun after this change.

The authoritative weighted audit is
`data/experiments/2026-08-28-phase21-weighted-solver-runtime-v3/`. It uses 32 independent
SLSQP/HiGHS problems, 1000 cold, repeated-warm and cycling-warm runs, and an O3 reference-host
binary compiled with all project warnings as errors. Maximum cold/dynamic total
setup+solve is `8.273542/8.790942 ms`; maximum hard/equality residual is `1.128e-7` and
stationarity is `4.124e-8`. Because the weakly regularized task problem has flat numerical
directions, equivalence accepts either scaled-variable equality or physical output plus
objective equivalence: maximum torque difference is `3.075e-5 N·m` and objective gap is
`1.896e-7`, below `5e-4 N·m` and `2e-6` gates. The hard corpus independently PASSes with
maximum cold/dynamic time `1.375261/1.103027 ms`.

From `ros_ws/`, the final package build and five CTest targets pass with zero errors,
failures or skips. A build alone is not the evidence; the golden and solver results above
are the acceptance authority.
