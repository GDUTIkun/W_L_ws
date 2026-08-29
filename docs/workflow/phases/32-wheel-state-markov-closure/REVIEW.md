# Phase 32: Wheel-State Markov Closure and Constrained-Dynamics Derivation — REVIEW

Status: `review`

Verdict: `REWORK`

## Findings

1. Phase31 measurement semantics reproduce, but its controlled dynamics replay used the XML
   `base_weld`; that dynamics authority is invalid for the floating production plant.
2. The corrected independent floating-base oracle passes with maximum epsilon cross-error
   `3.53e-10 m/s²`.
3. Controlled same-x16/same-request pairs prove `P32-D`, `P32-E` and `P32-F`; closure is
   `P32-C / M5`, not M4-only.
4. The current faceted wheel collision makes wheel angle a discrete contact-patch state. This agrees
   with the Phase21 historical non-differentiability result and blocks a smooth x16 repair.
5. x24 is the minimum evidence-backed necessary observable superset, but sufficiency is not proven.
   No production code, OCP, cost, WBC task or solver was changed.

## Verdict

`REWORK — P32-C_16state_markov_closure_failure (P32-D/P32-E/P32-F)`

Do not implement an x16 constrained-dynamics candidate. First validate the analytic round-wheel
contact authority (recommended) or explicitly accept a hybrid x24 mesh-phase model. T11–T14 remain
blocked and no `RECORD.md` is permitted.
