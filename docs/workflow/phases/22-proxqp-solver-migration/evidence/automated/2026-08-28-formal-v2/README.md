# Phase 22 authoritative formal v2

Result: `PASS` — 19/19 normal/perturbation and 6/6 fault cases.

This run supersedes `2026-08-28-formal-v1`. The v1 simulation result passed,
but its inherited manifest retained obsolete Phase 21 ADMM metadata. The
formal overlay now replaces the `solver` block as one identity object; this
manifest contains only ProxSuite ProxQP v0.7.3 fields.

## Worst authoritative results

- Core step: `0.978864 ms <= 10 ms`
- hard/primal: `1.6308412e-8 / 1.6308412e-8`
- dual/stationarity: `9.0499596e-9 / 9.0499528e-9`
- solver iterations: `6`
- normalized slack: `3.7278258e-3`
- task residual/cost: `5.2517504e-3 / 4.2896936e-5`
- all six fault cases: fail-zero, latch and reset checks PASS

## Fresh replay and integrity

- fresh replay: `2026-08-28-formal-v2-replay`, 19/19 + 6/6 PASS
- 25 plant CSVs are byte-exact
- control CSVs differ only in 22,379 `core_step_ns` wall-clock cells
- summaries are equal after removing `maximum_core_step_ms`
- primary and replay each have 71 checked config/runner/wrapper/scene/source/
  output hash entries; no mismatch
- rerunning against this non-empty directory exits 2 and leaves all 53 files
  unchanged
- manifest solver keys contain no `rho`, `sigma`, relaxation or other ADMM
  identity

## Historical regressions

Fresh Phase 22 namespaces under `data/experiments/` passed:

- Phase 14 internal dynamics: `overall_pass=true`
- Phase 15 closed-chain kinematics: `overall_pass=true`
- Phase 18 contact/floating-base: `overall_pass=true`
- Phase 20 3D standing: `pass=true`
- MuJoCo coordinate contract: PASS

This is current-nominal simulation-host evidence only. It does not establish
NMPC, identified-plant, real-machine or target-hardware real-time behavior.
