# Phase 35: Wheel-Position Servo Workspace Failure Attribution — RECORD

Status: `complete`

Review: [PASS](REVIEW.md)

## Outcome

Phase35 closes the Phase34 `kOutsideWorkspace` mechanism as
`P35-A_pre_target_minimal_wbc_workspace_drift`. The exact trigger is canonical index 5
(`right_wheel`) crossing its lower `-1 rad` delta bound at H0 tick 88, driven by pre-target bilateral
wheel-spin drift under the Phase27 Minimal fixed-wrench hold.

## Delivered and verified

- Behavior-invariant shared workspace inspector and exhaustive equality/precedence test.
- Append-only Phase35 runner/config/evaluator with rejecting-tick canonical and raw MuJoCo geometry.
- H0/H1 double fresh replay, six exact Phase34 tracking replays and formal-v2 authority.
- Contact, hard, slack and torque precedence exclusions; no justified paper-task mapping for absolute
  wheel spin.
- Build PASS; 35 tests PASS; fresh replay numeric error zero.
- Graphify code relationships were incrementally refreshed; Phase35 semantic-document ingestion is
  deferred after a provider HTTP 402 and is recorded in `graphify-out/.phase35_run.json`.

## Preserved boundaries

Production remains Phase27. No controller mode, NMPC/WBC/planner law, gain, bound, solver, task,
public message or hardware path changed. Direct pulses and H2 servo holds were intentionally not run
after P35-A made them causally ineligible.

## Next experiment

Run one offline fixed-state wheel-spin/mesh-phase validity sweep without bypassing the live gate in
closed loop. It must decide whether the `±1 rad` interval is a necessary rotating-mesh validity domain
or candidate P35-I contract issue; Phase35 makes neither conclusion.

## Key links

- [PLAN](PLAN.md)
- [REVIEW](REVIEW.md)
- [formal-v2 summary](evidence/automated/workspace-attribution-formal-v2/summary.json)
- [ROADMAP](../../ROADMAP.md)
