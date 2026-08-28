# Phase 22: ProxQP solver migration — REVIEW

Verdict: `PASS`

## Scope Review

- dependency/build: PASS. Exact ProxSuite v0.7.3 is consumed through
  `find_package(... EXACT CONFIG REQUIRED)` and `proxsuite::proxsuite`; clean
  Release packages-up-to build passed and missing dependency fails configure.
- solver adapter: PASS. `DenseQpSolver` retains the bound-form project boundary,
  performs exact equality/inequality row splitting, uses dense
  `PrimalDualLDLT`, and has no old ADMM fallback or settings.
- safety/lifecycle: PASS. Non-success candidates are exact zero; warm update is
  limited to a previous success with the same equality mask; reset restores a
  cold path. The WBC authority guard remains 42 variables, 104 rows and 12
  equalities.
- WBC/plant scope: PASS. Model, objective, constraints, tasks, weights,
  reference, scale, torque extraction, canonical I/O, Adapter, plant and
  2/10 ms timing remain the Phase 21 definitions.
- claim boundary: PASS. No NMPC, real-machine, identified-profile or target-CPU
  real-time conclusion is made.

## Evidence Review

Final authority is [formal-v2](evidence/automated/2026-08-28-formal-v2/summary.json),
with [manifest](evidence/automated/2026-08-28-formal-v2/manifest.json) and
[fresh replay-v2](evidence/automated/2026-08-28-formal-v2-replay/summary.json).
Formal-v1 and its replay are preserved but superseded because their manifests
contained inherited ADMM metadata; their simulation checks were not used as
final identity authority.

| Gate | Result | Evidence |
| --- | --- | --- |
| Dependency | PASS | v0.7.3 commit/config/header identity recorded; exact exported target consumed; missing dependency configure exits 1 |
| Build/tests | PASS | clean Release packages-up-to build; current ROS result `24 tests, 0 errors, 0 failures` |
| Component/failure | PASS | mixed/equality/bound/unconstrained, dimension, invalid/nonfinite/nonconvex/infeasible/limit, mask, cold/warm/reset paths; failed candidate zero |
| 32-case oracle | PASS | physical torque error `2.8314025e-5 N m`; objective gap `2.4415758e-9`; independent stationarity `2.9134814e-9` |
| 1000-run timing | PASS | cold/repeated-warm/dynamic maxima `0.929801/0.545467/0.689053 ms`; 10 ms component gate |
| Normal/perturbation | PASS | authoritative v2 19/19; hard/primal/dual/stationarity `1.631e-8/1.631e-8/9.050e-9/9.050e-9` |
| Fault/reset | PASS | 6/6 fault cases; fail-zero/latch/reset checks and double-episode replay PASS |
| Deadline/runtime | PASS | maximum Core step `0.978864 ms`; ZOH, Adapter sign, solver/task/plant gates PASS |
| Replay/integrity | PASS | 25 plant CSVs byte-exact; control only wall-clock differs; summaries equal without wall-clock; 142 total manifest hash entries match |
| Non-overwrite | PASS | existing authoritative directory exits 2; all 53 files unchanged |
| Compatibility | PASS | Phase14/15/18 `overall_pass=true`, Phase20 `pass=true`, coordinate contract PASS; Phase 21 evidence preserved |

## Grounding Review

CBM was refreshed to generation `2026-08-28T07:49:08Z`. Current graph snippets
and inbound traces confirm the project-owned `setup/solve/reset` boundary is
consumed by `WeightedWbcController::step` and the weighted-WBC runner path.
The solver/header/controller/CMake paths have current metadata with no recorded
coverage issue. `test_dense_qp_solver.cpp:29` remains a known parse-partial
range and was read directly; `tools/` and `docs/` are excluded by project
policy and were reviewed from live source. Coverage is best-effort, so build,
tests, manifests and actual formal outputs remain the acceptance authority.

Graphify history confirms Phase 21 formal-v1 remains the inherited WBC
authority and Phase 22 changes only the solver layer. The Graphify graph is
updated after this PASS record; no historical graph result substitutes for
the live evidence above.

## Findings

Blocking findings: None.

Non-blocking limits:

- Results cover only the frozen current-nominal full-3D MuJoCo case matrix and
  the reference simulation host.
- Component and formal timings do not establish Raspberry Pi, STM32 or hard
  real-time performance.
- ProxSuite remains an external exact-version build dependency; deployment
  environments must provide the same consumable CMake package.

## Conclusion

DG22-01 through DG22-05 are closed. The internal production QP backend is now
ProxSuite ProxQP v0.7.3 without changing the Phase 21 WBC mathematics or public
robot boundary. All required component, oracle, build/test, full formal,
fresh-replay, non-overwrite and historical compatibility evidence passed, so a
RECORD may be created and Phase 22 may become complete.
