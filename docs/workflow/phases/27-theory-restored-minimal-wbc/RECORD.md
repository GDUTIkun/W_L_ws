# Phase 27 Record

Status: `complete`

Review: `PASS` with 0 blocking findings.

Controller outcome: `Minimal FAIL` with bounded per-case attribution.

## Delivered

- canonical left/right/common/differential wheel-state and bounded common
  planner contract
- wheel-centre internal wheel-on-body interaction-wrench contract and affine
  reconstruction inside the unchanged 42D WBC variables
- current-parameter 16-state relative-rotation-vector model with two 10 ms RK4
  substeps and checked-in acados SQP-RTI/HPIPM v2 artifact
- retained 2/10/20 ms plant/WBC/NMPC schedule
- opt-in Minimal WBC profile with unchanged 104 hard rows and only wrench
  realization/slack, soft contact acceleration and weak regularization
- additive Phase 27 runtime, reference profiles, diagnostics, evaluator,
  immutable T0--T3 traces, failure packages and provenance manifests

## Results

- all model/interface/OCP/WBC component gates: PASS
- final Release regression: 32/32 PASS
- formal maximum combined time: `5.211306 ms < 10 ms`
- fresh replay: plant byte-exact; control exact except three wall-clock fields
- fault/reset/non-overwrite: PASS for solver failure, late, stale and non-finite
- T0 first failure: safety envelope at 0.58 s
- T1 and both T2 cases: safety envelope at 0.45 s
- T3 `+/-10 mm`: native NMPC stationarity audit at 0.04/0.08 s

## Decisions

- the Minimal candidate is not approved for closed-loop use
- T0--T2 show a controller-level base/reference stabilization gap after state
  tasks are removed; this Phase does not decide the corrective architecture
- T3 shows that the frozen single-RTI OCP lifecycle is not robust to the
  approved differential offset; no threshold was relaxed
- Phase 23 and all earlier default modes/evidence remain authoritative and
  non-overwritten
- no add-back task, solver retuning, plant change or real-hardware claim is
  made in this Phase

## Evidence

- [Formal method](evidence/formal-method-v1.md)
- [Formal outcome](evidence/formal-outcome.md)
- [Authoritative formal v2](evidence/automated/phase27-minimal-formal-v2/summary.json)
- [Fresh replay](evidence/automated/phase27-minimal-formal-v2-replay/summary.json)
- [Fault/replay verification](evidence/automated/phase27-fault-replay-v1/summary.json)
- [Runtime integration](evidence/runtime-integration.md)
- [16-state OCP](evidence/wheel-aware-acados-ocp.md)
- [Minimal WBC contract](evidence/minimal-wbc-contract.md)

## Next questions

A later Phase may separately test the smallest approved stabilization
architecture for T0--T2 and the OCP lifecycle needed for T3. Those questions
must start from this Phase's physical contracts and frozen failure evidence;
they must not infer task necessity from the retired empty audit handoff or claim
that either observed gap already proves a specific solution.
