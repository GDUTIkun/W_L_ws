# Phase 34: 12D Base NMPC + WBC Wheel Tracking Feasibility — REVIEW

Verdict: **REWORK**

## Outcome

The Phase27-semantics x12 model/OCP and the isolated longitudinal WBC task are numerically valid.
Gain-free common/differential physical authority is strong and well conditioned. The composed
planner-to-WBC wheel-position loop nevertheless fails the first tracking gate: every predeclared
gain set exits the valid WBC workspace before completing its step or ramp and remains outside the
tracking-error/settling limits. The proposed architecture is therefore not established feasible.

## Evidence

- DG34-01 model oracle: next `6.0694e-9`; Jx/Ju/Jxi `5.5777e-9 / 5.9128e-9 / 1.9125e-7`.
- DG34-02 OCP formal-v3: RTI `1.592 ms`, defect `4.2817e-4`, projected stationarity `0.03014`;
  converged SQP feasibility/stationarity `2.4911e-10 / 9.6195e-10`.
- DG34-03: self authority `>=0.70094`, cross/self `<=0.01304`, condition `<=1.35077`, realized-
  wrench change `<=0.1722%`, hard violation `<=1.37e-8`.
- DG34-04: all 6 runs rejected `kOutsideWorkspace` at ticks 90--92; final available common error
  `4.195--9.580 mm` versus `1 mm`, with no qualifying `0.2 s` settling.
- Release build passed. `colcon test` passed: 35 tests, 0 errors/failures, including the new Phase34
  affine/profile component and existing Phase27 regressions.

## Blocking Finding

`P34-E_wheel_tracking_failure`: local acceleration authority does not translate into an admissible
small-signal position loop on the current nominal plant under the frozen planner/task/gain screen.
Changing gains, workspace semantics or feedback structure after observing this gate would be a new
experiment, not completion of Phase34.

## Scope and Production

- No slack/workspace feedback, state augmentation, Q/Qe/R retuning, Delta-u term or Eq.(12).
- No Phase34 ControllerCore mode or production solver selection; production remains Phase27.
- T08--T11 are blocked and no Phase34 RECORD is created.

## Recommended Next Direction

Before proposing another controller, attribute which physical coordinate reaches the live WBC
workspace boundary and whether the drift exists during the pre-target hold or is introduced by the
longitudinal task. Use an append-only, gain-independent state-envelope decomposition. Do not reopen
NMPC formulation or add workspace feedback until that plant/WBC boundary cause is measured.
