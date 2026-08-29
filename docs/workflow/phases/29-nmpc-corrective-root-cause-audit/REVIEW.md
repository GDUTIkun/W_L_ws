# Phase 29: NMPC corrective-action root-cause audit — REVIEW

Status: `review`

## Review Scope

- PLAN: [`PLAN.md`](PLAN.md)
- Reviewed worktree: Phase 29 namespaced offline generator/evaluator/config,
  append-only generated artifacts, formal-v1/replay-v4 and Phase 28 regression
  integration present in the shared worktree
- Reviewer and date: Codex, 2026-08-29

## Implementation Check

| PLAN Task | Delivered | Evidence | Result |
| --- | --- | --- | --- |
| T01–T04 | Authority/lifecycle reproduction and state/reference contract | [audit evidence](evidence/root-cause-audit.md), formal manifests and raw case JSON | PASS |
| T05–T06 | Exact objective/model diagnostics and isomorphic converged SQP oracle | generated v2 manifest, saved stage/KKT data, production-vs-SQP field audit | PASS |
| T07–T09 | T0/T1 single, pairwise, bound, reference and terminal counterfactuals | formal-v1 T0/T1 root-cause JSON | PASS |
| T10 | Unique classifications and limited T2 holdout | summary and T2 holdout JSON | PASS |
| T11 | Fresh replay, non-overwrite and repository regressions | replay-v4, build/test and Phase 28 oracle results | PASS |

## Validation Results

| Validation | Command / Procedure | Actual Result | Evidence |
| --- | --- | --- | --- |
| Dependency/compile preflight | `./.venv/bin/python` dependency probe and `python -m py_compile` | Python 3.10.20; MuJoCo 3.7.0; NumPy 2.2.6; SciPy 1.15.3; CasADi 3.7.2; PASS | formal manifests |
| Lifecycle authority | Replay full problem prefixes to target actions | T0 `7.77e-16`, T1 `7.22e-16` maximum wrench error | T0/T1 formal-v1 JSON |
| OCP equivalence | Compare production and offline `acados_ocp.json` plus generation manifest | dimensions/cost/constraints/discrete horizon/HPIPM equal; only declared SQP convergence fields differ | offline SQP v2 manifest |
| Root-cause formal | Run evaluator to a new v1 output root | T0=`P29-E`, T1=`P29-D`, summary PASS | `evidence/automated/phase29-root-cause-v1` |
| Fresh replay | Run frozen evaluator to v4 and compare five semantic files | byte-identical; summary SHA-256 `a86573…c2172`; explicit replay relations | `evidence/automated/phase29-root-cause-v4` |
| Release build | `colcon build --symlink-install --packages-up-to wheel_leg_mujoco ...` | 4 packages finished | build output |
| Repository tests | `colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco` | 33 tests, 0 errors, 0 failures, 0 skipped | test output |
| Phase 28 oracle | `./build/wheel_leg_core/test_phase28_nmpc_corrective` | PASS; T0 `+118.153/+18.2632`, T1 `-0.972159/-0.491522` | executable output |
| Non-overwrite | Target existing formal-v1 output root | rejected before write, exit 1 | evaluator output |

## Findings

### Blocking

None.

### Non-blocking

- The earlier offline SQP v1 artifact remains as a superseded pre-formal
  diagnostic artifact. Formal authority uses v2 and records all hashes.
- Replay-v2 is semantically valid but superseded because its manifest omitted
  replay relations. Replay-v3 preserves an acados dynamic-library environment
  gate failure and contains no semantic output; replay-v4 is authoritative.
  The only evaluator change between v2 and v4 is additive CLI/manifest metadata;
  v1/v2 already provide the exact-runner replay, and all five v1/v4 semantic
  outputs remain byte-identical.
- CBM generation `2026-08-29T06:47:42Z` has no recorded gap for the unchanged
  live solver/model/controller sources. Changed CMake/runner files and the
  untracked test/generated/config files were reviewed directly; `tools/` and
  `docs/` are excluded from the index by repository policy.

## Decision and Evidence Review

- Frozen decisions preserved: yes. Production SQP-RTI artifact, wrapper,
  cost/reference/bounds, WBC, timing, fault/reset and public interfaces are
  unchanged by Phase 29.
- Evidence sufficient: yes. Each primary classification has lifecycle,
  converged-solve, counterfactual and model/action evidence under frozen gates.
- Open issues requiring a new Phase: any corrective redesign, retuning, solver
  lifecycle change or added WBC task; none is approved here.

## Verdict

`PASS`
