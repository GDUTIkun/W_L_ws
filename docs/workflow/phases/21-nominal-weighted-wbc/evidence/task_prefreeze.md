# Phase 21 Weighted-Task Pre-freeze

Date: 2026-08-27  
Verdict: **REWORK**

## Passed prerequisites

- Model oracle v8 passes its declared local/static gates; model oracle v9 also passes its declared rolling-selector differential gates.
- Hard-QP prefreeze v5 passes equality, torque, normal/friction and acceleration layers, cross-oracle, failure rejection and the C++ deadline benchmark.
- The task runner preserves 2 ms physics, 10 ms control and five-step ZOH and rejects non-empty output directories.

## Preserved task runs

- v1 nominal: cold ADMM reaches the 4000-iteration limit at tick 116; fail-zero exposes the expected plant fall. The end-of-run NumPy serialization defect is retained in the run directory.
- v2: `rho=10` removes the early numerical failure, but the original wrench fidelity penalty (`100`) drives 4.60 cm X drift by 2 s and `3.65` wrench slack.
- v3/v4: increasing base-X authority while retaining the strong wrench penalty worsens the physical conflict; changing `rho` fixes convergence but not the trajectory.
- v5: reducing wrench-slack penalty to `1` passes all 2 s position/orientation/contact/plant/QP gates except the intentionally inapplicable 10 s settling gate. The full 10 s nominal run becomes infeasible at tick 272 and falls.
- v6/v7/v9: base-X gain/weight and damping sweeps do not remove the nonlinear failure; successful numerical solves before failure do not imply plant stability.
- model v9/task v8: replacing the fixed wheel-material COP with an equilibrium-calibrated world-offset selector passes its local oracle but also fails the nonlinear nominal run.

## Blocking finding

The equilibrium-calibrated single-force contact proxy is not a valid nonlinear rolling-contact authority. Both tested selector semantics can satisfy their local differential/static checks and still diverge from the compliant three-contact-per-wheel plant after rolling begins. Continuing to tune task weights would hide a model error inside slack or motion residuals.

P21-T03 and DG21-01/02 are reopened. P21-T06 is blocked, and production model/Core integration is prohibited until a state-dependent contact representation—derivable from canonical state without MuJoCo contact truth—passes the same local, hard-QP and 10 s nonlinear gates. No RECORD is permitted.

## 2026-08-27 attribution addendum

The fresh fixed 12-case attribution matrix and independent failure-window QP oracle are recorded in [failure_attribution.md](failure_attribution.md). They correct the previously unconnected task switch and its empty-set defect, but do not alter the v5 QP, solver, model, timing, weights, bounds, or gates.

The baseline tick-272 QP is independently feasible and solvable, so that event is an ADMM iteration-limit failure rather than mathematical infeasibility. Disabling wrench fidelity delays it only two ticks; all single tasks have the intended median direction; and the mesh-contact reduced generalized-force mismatch grows with COP error, rolling slip, wrench slack, and contact/base-X residuals before failure. The supported attribution is therefore contact/model mismatch upstream with a downstream numerical solver failure. DG21-01/02 remain `REWORK`, P21-T06 remains blocked, and no tuning or Core work is released.

## 2026-08-27 42D re-entry

This document remains the authoritative failure record for the historical 36D/single-force
task runs, but its final blocked status has been superseded by the continuous-contact,
base/contact-closing, 42D solver and layered-hard evidence.  DG21-01 through DG21-04 are now
closed, P21-T03 through P21-T05 are complete, and P21-T06 has re-entered `doing`.

Re-entry does not revive the old task implementation or numbers.  The next oracle must be a
new 42-column implementation built on the frozen 104-row hard problem.  Its structural
candidate is:

- six soft contact-acceleration rows at the analytic contact centers;
- one world-X anchor row, one base-height row and three world-axis orientation rows;
- four hip/knee posture rows; wheel angles remain cyclic and are controlled through the
  common base-X objective plus the two rolling rows already present in the contact task;
- twelve controller-FLU wrench-fidelity rows with physical residual
  `W_feasible - W_reference - slack`, preserving
  `W_feasible = W_reference + slack`;
- no slack in any hard row and no inherited 36D weight, gain, wrench penalty or solver tuning.

Physical normalization scales, nonnegative weights, KP/KD values, equilibrium-wrench
generation, ablation thresholds and the exact tuning/holdout profile remain P21-T06 evidence
decisions.  They must be frozen by a fresh 42D oracle and 10 s nonlinear runs before DG21-05
can close or P21-T07 can begin.

The superseding 42D algebraic entry oracle is recorded in
[task_prefreeze_42d_local.md](task_prefreeze_42d_local.md).  It passes its local matrix,
frame/sign, 32-case hard-compatibility and 54-direction gates, but intentionally leaves the
nonlinear tuning/holdout gate open.
