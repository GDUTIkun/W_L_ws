# Phase 41 RECORD

Status: **complete / REVIEW PASS**  
Date: 2026-08-30

## Decision

The production nominal WBC workspace contract is now:

```text
hip/knee: current finite workspace bounds enforced
wheel q: finite-only, raw unwrapped cyclic coordinate
wheel dq and all other RobotState validity: unchanged
```

The Phase40 diagnostic policy fork is retired. Wheel absolute angle no longer causes
`kOutsideWorkspace` in the normal model/controller API.

## Production H0 evidence

The corrected production path crosses the old right-wheel ±1 location at tick96 with model status
OK. It reproduces the Phase40 shadow exactly and stops at tick111 on right-wheel contact loss; all
frozen independent gates are valid through tick110. Therefore the workspace-contract defect is
closed and contact loss becomes the next real H0 blocker.

Classification: `P41-A_workspace_contract_corrected_contact_loss_reproduced`.

## Scope retained

No contact-loss repair, wheel-rate damping/task, xi task/gain, equilibrium-wrench correction,
planner/NMPC change or Phase34 tracking run occurred. Real-hardware mechanical/sensor authority
remains unresolved and is not altered by this nominal software decision.

## Next authorized work

The next technical Phase should be **Wheel-Spin Drift / Contact-Loss Causal Attribution**. It must
separate rolling-DOF equilibrium, wheel-rate drift, left/right asymmetry and base/leg/contact
coupling before choosing any minimal repair. Phase34 remains frozen until that causal chain and its
required correction are independently closed.
