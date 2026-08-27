# Phase 21 Solver Audit

Date: 2026-08-26  
Status: historical 36D result; superseded for current authority by `solver_audit_42d.md`

## Required Problem

Convex QP in the fixed form `min 0.5 z'Hz + g'z` subject to `l <= Az <= u`, with `z` fixed at 36 variables. Equality rows use identical finite lower/upper bounds. Candidate output is accepted only after independent finite, convexity, hard residual, inequality margin and iteration/deadline checks.

## Candidates

- **OSQP:** official documentation supports the exact bound-form convex QP, warm start, factorization reuse and primal/dual infeasibility statuses under Apache-2.0. It is not installed in the current workspace or reference host.
- **ProxQP/ProxSuite:** official project provides dense/sparse C++ QP paths under BSD-2-Clause and is robotics-oriented. It is not installed in the current workspace or reference host.
- **qpOASES:** dense online active-set C++ solver under LGPL-2.1. It is not installed, and adding an older LGPL dependency is unnecessary for this fixed small problem.
- **Eigen-only fixed dense ADMM:** Eigen 3.4.0 is already installed (`libeigen3-dev 3.4.0-4build0.1`, MPL-2.0). A small fixed-bound-form implementation needs one deterministic LDLT factorization plus projection and supports warm start without adding a solver dependency.

Official sources checked:

- https://osqp.org/docs/index.html
- https://osqp.org/docs/interfaces/status_values.html
- https://github.com/Simple-Robotics/proxsuite
- https://github.com/coin-or/qpOASES

## Frozen Path

Use a project-owned, Eigen-only dense ADMM component for Phase 21. This is an implementation of the published bound-form splitting pattern, not copied OSQP source and not branded as OSQP.

- Fixed dimensions/capacities; no allocation inside `solve` after setup.
- Symmetric PSD `H`, finite matrix/vector input and `l <= u` validated before factorization.
- Config freezes `rho`, `sigma`, absolute/relative tolerance and maximum iterations; no adaptive `rho` or polishing in this Phase.
- Warm start is explicit state. Reset/cold start is deterministic; warm start cannot change the accepted solution beyond frozen tolerance.
- `converged` only means the ADMM termination test passed. Production acceptance separately recomputes hard equality residual, bound violation, stationarity/dual diagnostics and the reference-host deadline.
- Invalid data, factorization failure, non-finite iterate, inconsistent bounds, maximum iteration or deadline status is rejected and maps to six zero torques plus the Core safety latch.
- No equality solve, clipped candidate or previous feasible command is an accepted fallback.

The historical component corpus passes in `test_dense_qp_solver.cpp`. The final v5 36D hard-QP evidence passed with maximum hard residual `5.80e-8`, stationarity residual `5.52e-8`, and rejected infeasible input. Its 1000-run C++ reference-host benchmark recorded cold p99 `1.80011 ms`, cold maximum `2.888224 ms`, and warm solve `0.00344 ms`. After DG21-01/02 selected the 42D contact-centred-wrench contract, none of these figures remained current authority. The replacement 42D audit is recorded in `solver_audit_42d.md`.
