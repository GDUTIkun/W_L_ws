# Phase 21 42D Dense ADMM Solver Audit

Date: 2026-08-27  
Status: DG21-03 component gate PASS; simulation-only reference-host authority

## Audited path

`DenseQpSolver` now has 42 fixed variables and capacity for 128 bound-form rows;
the frozen hard-QP uses 104. `setup` remains cold by default. An explicit warm
setup preserves ADMM state only when the previous setup succeeded and the row
count is unchanged. It refactorizes the changed normal matrix and reprojects the
retained auxiliary variable onto the new bounds. A changed row count falls back
to a reset, so stale row semantics cannot be retained accidentally.

The solver still performs no allocation inside `solve`, rejects invalid,
non-finite, inconsistent-bound and indefinite input, and returns a zero candidate
on rejection or iteration limit. Acceptance is external to the solver status and
recomputes equality, bound, stationarity, oracle-difference, and deadline gates.

## Independent corpus

The corpus comes from `validate_weighted_wbc_hard_qp_42d.py`: 32 independently
rebuilt 42D/104-row problems, covering four Phase-21 workspace states and 28
rolling dynamic ticks. Every problem carries the independent SLSQP solution;
HiGHS separately established mathematical feasibility. The benchmark equality
count is read from the corpus header and is 12, not the historical 36D count of
24.

## 1000-run reference-host result

All timings include problem setup/factorization plus solve. Each mode ran exactly
1000 times.

| Mode | p50 ms | p99 ms | max ms | mean iterations | max oracle difference |
| --- | ---: | ---: | ---: | ---: | ---: |
| cold, cycling corpus | 1.076581 | 1.448752 | 1.455389 | 171.236 | 3.7087e-7 |
| repeated same problem warm | 0.085851 | 0.090501 | 0.101160 | 1.000 | 1.0922e-7 |
| cycling dynamic warm | 0.909702 | 1.393420 | 1.556428 | 136.435 | 3.6760e-7 |

Across the three modes, maximum equality/bound violation is `8.72e-8` and
maximum reported stationarity residual is `7.38e-8`. The frozen gates are
`2e-7` for equality/bounds, `2e-6` for stationarity and oracle difference, and
`10 ms` for cold/dynamic total setup+solve. All pass.

The historical 36D result recorded cold p99/max `1.80011/2.888224 ms` and a
single same-problem warm solve `0.00344 ms`. This is retained only as context:
the 36D objective, rows, corpus, and timing boundary differ, so the observed
42D numbers do not prove an intrinsic speedup. The current 42D corpus and
total-setup timing are the only DG21-03 authority.

## Failure corpus and build verification

The C++ component tests cover the final 42nd dimension, analytic golden QPs,
active equality/lower/upper bounds, deterministic repeated cold start,
same-problem warm start, changed-matrix/bounds warm start, changed-row reset,
non-finite data, non-convex Hessian, inconsistent bounds, and iteration limit.

`colcon build --packages-select wheel_leg_core --cmake-args -DBUILD_TESTING=ON`
and `colcon test --packages-select wheel_leg_core` pass; `colcon test-result
--verbose` reports 20 tests, zero errors and zero failures. The standalone
benchmark also compiles with `-Wall -Wextra -Wpedantic -Werror`.

DG21-03 is therefore closed for Phase-21 simulation work. This does not close
DG21-04, validate equilibrium/hard margins over the P21-T05 layer matrix, freeze
weighted tasks, authorize Core integration, or establish target-hardware timing.
