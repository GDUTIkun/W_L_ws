# Phase 33: Low-Dimensional Closure Recovery via WBC Manifold Regulation — REVIEW

Verdict: **REWORK**

## Outcome

Phase33 established a correct, independent and opt-in two-side wheel-height acceleration task without
changing the 42D/104-row WBC or production selection. Gate0, coordinate/affine parity and algebraic
invariance pass. The first gain-free physical authority screen does not pass the predeclared
cross-side isolation gate, so execution stops before any gain, closure retest, bandwidth test, contact
revision, rollout or NMPC recheck.

## Evidence

- Phase32 fresh authority summaries are byte-identical to their final replays.
- `ddzeta=A*nudot+b` central-derivative error: `1.7180701306e-12 m/s^2`.
- zeta row vs existing contact-row span residual: `0.8969324949`; the task is distinct.
- Phase33 component: A/lower/upper exact Minimal parity; no dimension/hard-row change; non-finite
  input rejects; production profile remains Phase27.
- Formal/replay (`zeta-authority-v3/v4`) byte-identical:
  self gain min `0.6186879390`, cross/self max `0.5125767989` (gate `0.5`), wrench change max
  `0.0034090934`, consistency max `0.0002001691`, hard max `1.4999489e-8`.
- Release `colcon build` PASS; `colcon test` PASS, 16 tests / 0 failures.

## Blocking Finding

`unresolved`: gain-free authority is material and wrench preservation passes, so neither P33-A nor
P33-B is truthful. One C1 state narrowly violates the frozen cross-side isolation gate. Changing the
gate or choosing gains after observing this result would be post-hoc tuning.

## Scope and Production

- No NMPC state/cost/reference/horizon/solver or safety change.
- No `kp/kd`, C1/C2 regulated closure evidence, bandwidth evidence or round-wheel model.
- `kPhase33ZetaManifold` is diagnostic-only and not selected by ControllerCore.
- Production remains Phase27. No RECORD is created.

## Recommended Next Direction

Use a new append-only experiment, not a silent continuation of this failed gate: characterize the
full 2x2 physical authority matrix and predeclare conditioning/decoupling and wrench gates. If a
coordinate-coupled soft objective cannot deliver fast attraction without wrench loss, stop pursuing
manifold elimination and evaluate the smallest observable normal-coordinate/rate augmentation. Only
after that decision should smooth round-wheel spin/phase separation resume.
