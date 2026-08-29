# Phase 27 Formal Outcome

Date: 2026-08-29

Decision: `DG27-06 CLOSED — Minimal FAIL with bounded attribution`

## Authority

The authority is `automated/phase27-minimal-formal-v2`. The v1 trajectories
are retained, but their evaluator used the previous tick's motion reference
when classifying the moving x envelope and allowed latch-zero diagnostics to
look like T3 recovery. It is therefore evaluator-inconclusive. V2 changes no
model, solver, case, duration, initial condition or threshold.

All six v2 cases completed their requested log duration but failed the hard
controller gate after strict zero/latch. The maximum pre-latch combined timing
remained `5.211306 ms < 10 ms`; maximum WBC hard violation was `4.213e-9`,
maximum independent NMPC defect `8.049e-5`, and maximum projected stationarity
`0.028325`. Thus the common T0--T2 failure is not a deadline, QP feasibility,
generated-model defect or independent stationarity failure.

## Per-case first failure

| Case | First tick/time | Unique first layer | Evidence |
| --- | ---: | --- | --- |
| T0 static | 58 / 0.58 s | safety envelope | base did not remain within the frozen x tracking envelope; NMPC/WBC audits were healthy at the first failure |
| T1 start-cruise-brake | 45 / 0.45 s | safety envelope | base did not follow the advancing x anchor without a state task |
| T2 left | 45 / 0.45 s | safety envelope | same longitudinal tracking loss; turn-specific metrics never became admissible after the hard failure |
| T2 right | 45 / 0.45 s | safety envelope | symmetric case reaches the same first layer |
| T3 `+10 mm` | 4 / 0.04 s | NMPC OCP audit | native acados stationarity `1.45856 > 1.0`; independent defect/bounds/projected stationarity remain within gate |
| T3 `-10 mm` | 8 / 0.08 s | NMPC OCP audit | native acados stationarity `1.60873 > 1.0`; independent defect/bounds/projected stationarity remain within gate |

The reconstructed T3 initial values were
`+0.0100000000000012 m` and `-0.0100000000000007 m`, inside the frozen
`1e-6 m` tolerance. No differential WBC task assisted either run.

## Interpretation boundary

Component evidence still closes wheel-state/planner, internal interaction
wrench, 16-state dynamics/generated parity and Minimal WBC algebra. The
controller-level candidate does not pass T0--T3. For T0--T2, the first missing
behavior is fast base/reference stabilization after removal of state tasks.
For T3, the current single-RTI OCP lifecycle is not robust to the approved
differential initial offset. These are next-Phase questions, not permission to
add tasks or retune this Phase.

Fresh-process replay reproduced every plant CSV byte-for-byte and every
control field except the three declared wall-clock columns. The four NMPC
fault classes passed exact-zero latch/reset replay, and output non-overwrite
passed. Final Release regression passed 32/32 tests.
