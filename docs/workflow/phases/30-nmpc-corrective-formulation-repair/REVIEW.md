# Phase 30 v3: reference-consistent NMPC formulation audit — REVIEW

Status: `review`

Verdict: `REWORK`

## Review Scope

- Preserved reviews: v1 direct-weight and v2 structured-cost REWORK.
- Reviewed: exact Phase29 authority replay, full-horizon reference defect, bounded best-input defect,
  conditional Branch-M recorded-MuJoCo prediction and fresh replay.
- Not entered by gate: new feedforward, cost/terminal candidate, local candidate RTI/SQP,
  closed-loop integration or production artifact.

## Findings

1. **Reference/feedforward inconsistency is not supported.** T0/T1 current reference defects are
   already below the pre-frozen small threshold at all six update problems.
2. **The 16-state model has a local wheel-state adequacy failure.** At 20 ms, individual wheel-rate
   prediction errors are `0.0194..0.0340 m/s` (`0.1296..0.2258` normalized), exceeding the `0.1`
   gate. Requested- and realized-wrench predictors agree closely, so WBC realization is not the
   explanation for this discrepancy.
3. **The conclusion is deliberately narrow.** Base-state groups are much closer at 20 ms; the
   evidence points to wheel-center relative kinematics/dynamics/state semantics. The 200/400 ms
   recorded trace is unavailable after safety latch, but the local 20 ms rejection is already
   sufficient to block formulation integration.

## Verdict

`REWORK — P31-F_wheel_state_model_adequacy_failure`

Do not tune cost, add stage feedforward, generate a production artifact, create `RECORD.md`, or mark
Phase 30 complete. Repair and independently validate the wheel-state model/measurement contract first.
