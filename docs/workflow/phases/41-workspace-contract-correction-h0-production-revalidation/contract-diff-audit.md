# Phase 41 contract-only diff audit

DG41-00: **PASS**.

Production contract now has one path:

```text
NominalWbcModel::inspectWorkspace
  hip/knee entries (0,1,3,4): bounds determine minimum/first failure/inside
  wheel entries (2,5): historical margins remain telemetry only

NominalWbcModel::evaluate
  validateRobotState first (q/dq finite)
  reject only when inspectWorkspace().inside() is false
```

Phase40’s temporary `WheelWorkspacePolicy` enum and parameters were deleted from both
`NominalWbcModel` and `WeightedWbcController`; no bypass/default ambiguity remains. Existing callers
again use the single `evaluate(state)` / `step(state, reference)` API.

Allowed supporting changes are limited to:

- regression assertions for wheel 2π/10π acceptance, wheel Inf rejection and unchanged leg bounds;
- separately compiled `phase41_workspace_contract_loop` from the existing Phase35 loop source;
- Phase41 config/runner/docs/evidence.

No WBC problem/task, controller reference, gain, wrench, planner, NMPC, contact, friction, torque,
model geometry/mass/inertia or hardware code changed. Direct source search shows the per-entry wheel
margin telemetry has no live consumer outside logging/tests; production verdict uses only
`WorkspaceInspection::inside()`.
