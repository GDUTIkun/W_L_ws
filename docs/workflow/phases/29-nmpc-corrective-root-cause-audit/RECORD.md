# Phase 29: NMPC corrective-action root-cause audit — RECORD

Status: `complete`

Review: [PASS](REVIEW.md)

## Outcome

Phase 29 uniquely closes T0 static as
`P29-E_horizon_reference_propagation` and T1 straight as
`P29-D_cross_state_coupling` without changing production control.

## Delivered

- Frozen method: [phase29_nmpc_root_cause_v1.json](../../../../simulation/mujoco/config/phase29_nmpc_root_cause_v1.json)
- Offline evaluator: [run_phase29_nmpc_root_cause.py](../../../../tools/experiments/run_phase29_nmpc_root_cause.py)
- Append-only SQP generator: [generate_phase29_acados_oracle.py](../../../../tools/experiments/generate_phase29_acados_oracle.py)
- Reviewable evidence: [root-cause-audit.md](evidence/root-cause-audit.md)
- Formal authority and replay: `evidence/automated/phase29-root-cause-v1` and
  `phase29-root-cause-v4`

## Verification Evidence

- T0/T1 production prefixes reproduce authoritative requested wrenches within
  `7.77e-16` and `7.22e-16`; state/reference errors are `3.47e-18` and zero.
- Production, cold, repeated RTI and converged SQP all retain the
  non-restorative direction, excluding lifecycle/RTI artifact as primary.
- T0 becomes restorative only when the terminal objective or specifically its
  base-longitudinal component is removed; held-reference and bound shadows do
  not. This isolates finite-horizon terminal propagation.
- T1 remains non-restorative for terminal/reference/bound shadows but becomes
  restorative when attitude cost is removed. Pairwise and acceleration
  decomposition evidence closes attitude-dominant cross-state coupling, with
  wheel-rate interaction secondary.
- T2 right matches the T1 mechanism; T2 left is already restorative and is
  `not_same`. No T2 primary attribution is added.
- Formal-v1 and replay-v4 have five byte-identical semantic outputs; summary
  SHA-256 is `a86573b537bbbd783c1ff8c78ff7f64d41b531b2af0e2bbd7256870c5f9c2172`.
- Release build PASS; suite reports `33 tests, 0 errors, 0 failures, 0 skipped`;
  the Phase 28 oracle reproduces its frozen derivatives; non-overwrite retry
  is rejected.

## Decisions Confirmed

- T0 primary cause: `P29-E`, specifically terminal base-longitudinal objective
  propagation through the finite-horizon coupled model.
- T1 primary cause: `P29-D`, specifically attitude-dominant cross-state
  coupling with a secondary wheel-rate interaction.
- Cold snapshot, actual production lifecycle and converged offline SQP remain
  distinct solve semantics.
- Phase 29 approves attribution only, not tuning, solver changes or WBC tasks.

## Deviations from PLAN

- The SQP oracle required continued iterations to satisfy the frozen `1e-7`
  stationarity gate; the gate was not relaxed. The pre-formal v1 generated
  artifact is retained and final formal authority uses v2.
- Replay-v2 was superseded only to add explicit replay-relation metadata;
  replay-v3 preserves an environment-gate failure, and replay-v4 is the fresh
  authoritative replay. No existing run was overwritten.
- Existing generated acados APIs and offline replay avoided a production
  diagnostic accessor.

## Known Limitations and Follow-ups

- Evidence is current-nominal and simulation-only; it does not validate an
  identified plant or real hardware.
- Any corrective design or tuning requires a separate Phase.

## ROADMAP Update

- Corresponding entry: Phase 29, NMPC corrective-action root-cause audit
- Status change: `review -> complete`
- Next executable Phase: none without a new simulation decision; Phase 05
  remains blocked by the real-hardware freeze.

## Key Links

- [PLAN](PLAN.md)
- [REVIEW](REVIEW.md)
- [ROADMAP](../../ROADMAP.md)
