# Phase 30 v1: NMPC corrective-action formulation repair — REVIEW

Status: `review`

Verdict: `REWORK`

## Review Scope

- PLAN: [`PLAN.md`](PLAN.md)
- Reviewed: authority/method freeze, independent runtime running/terminal cost scales, all-one
  parity, T0/T1 zero screens, fixed alpha/beta grids, KKT/branch checks, replay and non-overwrite.
- Not implemented by gate: isolated/combined closed loops, static artifact, production C++ audit
  switch, T2/T3/fault/deadline/history regressions.
- Reviewer and date: Codex, 2026-08-29.

## Implementation Check

| PLAN Task | Delivered | Evidence | Result |
| --- | --- | --- | --- |
| T01–T02 | Phase 29 handoff, frozen method/schema/grid/margins and no-retune rules | PLAN and method JSON | PASS |
| T03 | Runtime `W/W_e` decomposition, independent recomputation and all-one authority parity | [causality evidence](evidence/repair-causality.md), formal JSON | PASS |
| T04 | T0 zero screen and complete alpha grid | formal-v1/v2 | FAIL — R30-A |
| T05 | T1 direct-weight zero screen and complete beta grid | formal-v1/v2 | FAIL — R30-B |
| T06–T11 | PLAN forbids execution without selected alpha and beta | no candidate exists | BLOCKED |
| T12 | Evidence index, replay/non-overwrite and review input | causality evidence and this review | PASS |

## Validation Results

| Validation | Actual result |
| --- | --- |
| Dependency/compile preflight | Python 3.10.20; NumPy 2.2.6; SciPy 1.15.3; MuJoCo 3.7.0; CasADi 3.7.2; acados import, `py_compile`, JSON parse PASS |
| All-one baseline | T0/T1 production and converged `u0`, plus converged objective, exact; prefix errors `7.77e-16/7.22e-16` |
| T0 alpha=0 | Current-point pitch C positive, but all pitch D negative and all x/vx guards negative; FAIL |
| T0 fixed grid | No eligible scale; R30-A |
| T1 beta=0 | Authority tick x/vx C negative; pitch angle/rate C and D negative; FAIL |
| T1 fixed grid | No eligible nonzero scale; R30-B |
| Fresh replay | Both semantic files byte-identical between v1 and v2 |
| Non-overwrite | Existing formal-v1 root rejected before write |

## Findings

### Blocking

1. **B01 — terminal-x scalar is insufficient.** Removing only terminal x weight changes the
   current action but leaves the local pitch feedback derivative anti-corrective and reverses x/vx
   net action. Phase 29's terminal `[x,vx]` group shadow did not authorize this single weight.
2. **B02 — running-attitude scalar is insufficient.** Removing running attitude weight is not
   equivalent to setting attitude error to reference. It fails to restore the T1 authority action
   and retains anti-corrective pitch response. No nonzero beta passes.
3. **B03 — integration gates are unavailable.** With neither selected scalar, isolated/combined
   closed loop and production artifact work cannot start without violating the PLAN.

### Non-blocking

- The first smoke lacking the acados dynamic-library path stopped before output creation and is an
  environment failure, not model evidence. Both formal runs used the frozen repository interpreter
  and correct acados runtime path.
- Production solver, C++ wrapper/audit, WBC and public interfaces remain unchanged. Repository-wide
  build/test was deliberately not used to infer control PASS after the pre-integration gate failed.

## Decision and Evidence Review

- Frozen decisions were preserved: yes. The evaluator used only alpha and beta on the frozen grid;
  no third weight, interpolation, threshold relaxation, WBC task or production switch was added.
- Evidence sufficiency: sufficient to reject both scalar hypotheses and require REWORK; insufficient
  to select a replacement formulation.
- Next technical decision: a new formulation-design Phase must decide whether terminal
  base-longitudinal responsibility and attitude/longitudinal coupling require structured changes.
  That decision must not be hidden as further Phase 30 tuning.

## Verdict

`REWORK`

Do not create `RECORD.md` and do not mark Phase 30 complete. Preserve v1/v2 evidence. Any next attempt
must start from a new approved formulation boundary rather than extending this alpha/beta grid.
