# Phase 30 v2: structured NMPC formulation redesign — REVIEW

Status: `review`

Verdict: `REWORK`

## Review Scope

- Current PLAN: [`PLAN.md`](PLAN.md), V2 Structured Formulation Route.
- Preserved v1 review: [`REVIEW-v1-2026-08-29-REWORK.md`](REVIEW-v1-2026-08-29-REWORK.md).
- Reviewed: full-matrix cost oracle, baseline parity, frozen A1/A2/B2±/B4 candidates, PSD and
  conditioning, converged KKT/active branch, production lifecycle direction, replay/non-overwrite.
- Not entered by gate: isolated/combined closed loop, static production artifact, T2/T3 and
  fault/deadline/history regression.
- Reviewer and date: Codex, 2026-08-29.

## Task Review

| V2 task | Result | Evidence |
| --- | --- | --- |
| V2-T01–T02 | PASS | candidate spec, exact baseline parity, full-matrix cost/PSD/KKT oracle |
| V2-T03 | FAIL — R31-A | A1/A2 retain anti-corrective pitch derivatives and negative x/vx guards |
| V2-T04 | FAIL — R31-B | B2±/B4 retain non-restorative T1 action and pitch response |
| V2-T05–T07 | BLOCKED | no selected T0 or T1 candidate |
| V2-T08 | PASS for REWORK assembly | formal/replay, non-overwrite, evidence and state consistent |

## Blocking Findings

1. **B01 — terminal cost-only structures are insufficient for T0.** Removing terminal x/vx or
   keeping only a weak terminal velocity responsibility flips the finite RTI action, but does not
   make the local pitch feedback restorative and worsens the longitudinal action guards.
2. **B02 — tested cross-state structures are insufficient for T1.** Small PSD longitudinal-pitch
   correlations and removal of common wheel-rate responsibility do not reproduce the Phase 29
   attitude-state-removal counterfactual. The T1 authority action remains non-restorative.
3. **B03 — no integration authority.** The route explicitly requires both local candidates before
   closed loop or static artifact work. Continuing would be post-result candidate expansion.

## Decision Review

- Frozen plant/model/horizon/solver/WBC/fault/safety boundaries: preserved.
- Numerical validity: all candidates PSD; KKT, feasibility, objective recomputation and branch checks
  pass. Failures are causal-direction failures, not numerical-health failures.
- Evidence scope: sufficient for R31-A/R31-B on this predeclared bounded set, not an exhaustive
  impossibility result for all costs.
- Next decision must revisit reference/terminal-control responsibility or control architecture. It
  must not add more Phase 30 cost candidates after seeing these results.

## Verdict

`REWORK`

Do not create `RECORD.md` or mark Phase 30 complete. Production remains at the Phase 27 formulation.
