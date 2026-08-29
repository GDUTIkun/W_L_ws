# Phase 28 Record

Status: `complete`

Review: [PASS](REVIEW.md)

## Decision

Phase 28 closes T0 static and T1 straight first physical divergence as
`B_nmpc_corrective_failure` on the current nominal MuJoCo profile.

This classification means the requested interaction wrench, evaluated through
the frozen 16-state model, produces a net acceleration that reinforces the
relevant error in the first-divergence window. It does **not** mean the acados
solver failed, the WBC failed to realize its request, the plant disagreed with
the reduced acceleration, or a particular stabilization task is necessary.

## Evidence chain

- Phase 27 authority: `phase27-minimal-formal-v2`.
- Frozen method/config: [phase28_drift_attribution_v1.json](../../../../simulation/mujoco/config/phase28_drift_attribution_v1.json).
- Final primary run: `evidence/automated/phase28-drift-attribution-v5`.
- Fresh replay: `evidence/automated/phase28-drift-attribution-v6`.
- Normal-mode regression: `evidence/automated/phase28-phase27-normal-regression-v1`.
- Preserved invalid/superseded runs: `v1` through `v4`.

T0 reproduces nominal failure at tick `58` and stops diagnostically at tick
`70`; T1 reproduces tick `45` and stops at tick `62`. In both, WBC realization,
resource, acceleration-oracle, upper-model, and plant-match gates pass before
the decision tree reaches any later layer.

The frozen-state solver oracle adds the distinguishing causal evidence: T0
pitch and pitch-rate perturbations have positive acceleration derivatives
`+118.153` and `+18.2632`; T1 perturbation derivatives are locally negative,
but the unperturbed snapshot acceleration `-0.0118472 m/s²` reinforces its
negative position and velocity errors. This agrees with the trajectory-window
corrective scores.

## T2 and limits

T2 is a symmetry check only. Right turn follows the T1 B path; left turn does
not, so the result is `not_consistent`. Phase 28 assigns neither turn a primary
mechanism and makes no bandwidth, task-necessity, T3, identified-profile, or
real-hardware claim.

## Compatibility

Diagnostics are disabled by default and accepted only with the Phase 27
simulation runner mode. Phase 27 normal output reproduces its prior semantic
summary, fault/reset exact-zero behavior remains covered, and the 33-test ROS
suite passes. The old nominal safety failures and thresholds remain unchanged.
