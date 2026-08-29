# Phase 23 authoritative acados formal v2

Result: `PASS` — 23/23 normal/reference and 10/10 fault cases.

This is the first integrated authority using the append-only v2 generated
artifact with state bounds on stages 1 through N. Its manifest explicitly
supersedes the pre-freeze v1-v8 lineage. The conclusion is limited to the
current nominal MuJoCo simulation host.

## Worst authoritative results

- normal NMPC + WBC combined time: `3.641244 ms < 10 ms`
- independent dynamics defect: `2.25681e-6 <= 1e-3`
- projected stationarity: `0.0250671 <= 0.05`
- input/state bound violation: `0 / 0`
- maximum Core step: `3.641879 ms < 10 ms`
- WBC stationarity: `8.96584e-9`
- normalized slack: `0.00368988`
- task residual: `0.00544410`
- closure residual: `0.183516 mm`

All four NMPC-specific faults and the six inherited faults pass fail-zero,
latch and reset checks.

## Replay and integrity

- replay: `2026-08-29-phase23-acados-formal-v2-replay`, 23/23 + 10/10 PASS
- 33 plant CSVs are byte-identical
- 33 control CSVs are identical outside the four declared wall-clock fields
- summaries are equal outside profile identity and `maximum_core_step_ms`
- non-empty output is rejected with exit 2
- manifest records v2 generated inputs, source/config/output hashes, acados
  commit/library hashes, renderer, interpreter dependencies, loader resolution,
  `supersedes`, and `replay_of`

Fresh coordinate and Phase 14/15/18/20 regressions pass. Current component and
formal gates exercise the inherited Phase 21/22 WBC contract; their approved
configs and evidence remain unmodified.
