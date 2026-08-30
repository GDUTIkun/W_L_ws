# Phase 44: WBC-to-Plant Constrained Rolling Realization Audit — RECORD

Status: `complete`
Date: 2026-08-30

## Outcome

Phase44 closes as `P44-E — Multiple realization layers`. The initial review correctly rejected a
single symmetric central Jacobian (`P44-U/REWORK`). The append-only regime-aware addendum repaired
DG44-06 with exact contact/assembled-inequality signatures and trusted one-sided derivatives.

## Approved Evidence

- original formal/replay v1: task, affine acceleration, reduced contact, rolling-coordinate and C
  decomposition evidence;
- addendum authoritative formal/replay v4: regime signatures, directional classifications,
  trusted `G_QP/G_MJ/G_mis`, contact transfer and chronology;
- [REVIEW-addendum.md](REVIEW-addendum.md): all DG44-R1..R10 PASS.

Key results are 396 R44-S, 84 R44-P, no R44-O/B; trusted tick0 B authority `QP +1 -> MJ -0.509`;
trusted D/native-common contact cancellation ratio `0.761..0.902`; B/D persistent R44-P onsets at
ticks 98/110; C's xi response remains dominated by leg/wheel-center motion.

## Frozen Consequences

- Do not use an ordinary central Jacobian at R44-P states and do not interpret untrusted directions.
- Future rolling realization must be contact-consistent and account for xi, native spin and
  slip/load information; this RECORD does not mandate three independent tasks.
- The next repair work, if opened, must choose one mechanism-specific minimum repair. It may not
  infer a contact-solver/code defect from this evidence and must retain the no-tuning/no-plant-change
  boundary until its own PLAN freezes otherwise.
- No Phase34 tracking, planner/NMPC expansion, new repair rollout or Phase45 implementation occurred
  in Phase44.
