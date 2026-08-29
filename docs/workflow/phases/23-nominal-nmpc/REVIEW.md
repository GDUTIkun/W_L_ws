# Phase 23 Review

Verdict: `PASS`

Blocking findings: 0

## Reviewed scope

The review covers the approved current-nominal MuJoCo-only 12-state/12-wrench
NMPC, append-only acados v2 generated artifact, private C++ wrapper, opt-in
NMPC+Weighted-WBC Core mode, 2:1 schedule/ZOH, runner diagnostics, failure
semantics, formal/replay evidence and historical compatibility. It excludes
real hardware, identified plant, turning, large yaw, terrain and target-host
real-time claims.

## Gate review

| Gate | Result | Evidence |
| --- | --- | --- |
| State/model contract | PASS | state-oracle-v2 and model-oracle-v5; generated v2 parity next/A/B errors `4.86e-17 / 4.13e-11 / 3.11e-11` |
| acados toolchain/OCP | PASS | commit `21376cb1...`, fixed renderer, generated ABI/source manifest, loader resolves acados/HPIPM/BLASFEO |
| Cost/reference/constraints | PASS | real solver ablations, 4 tuning and 6 unseen nonlinear holdout; no optimizer-only memory augmentation |
| Generated component | PASS | stages 1..N state bounds, exact stage 0, normalized double generation, ordinary build without Python, cold/warm/reset and 3x1000 solves |
| Runtime contract | PASS | NMPC-before-WBC, 2:1 update/two-tick ZOH, diagnostics, reset, solver/late/stale/non-finite strict zero+latch |
| Build/tests | PASS | clean Release four-package build; `26 tests, 0 errors, 0 failures, 0 skipped`; warnings-as-errors targets pass |
| Integrated formal | PASS | authority v2 and replay each 23/23 normal/reference and 10/10 fault PASS |
| Deadline/OCP audit | PASS | normal combined max `3.641244 ms < 10 ms`; defect `2.25681e-6`; projected stationarity `0.0250671`; input/state bound violation `0/0` |
| WBC/plant gates | PASS | inherited hard/task/contact/slip/closure/ZOH gates all pass; max closure `0.183516 mm` |
| Replay/non-overwrite | PASS | 33 plant CSVs byte-exact; 33 control CSVs differ only in four declared wall-clock fields; non-empty output exit 2 |
| Provenance | PASS | primary/replay each 99 current source/generated/output hashes verified; explicit `supersedes/replay_of` |
| Compatibility | PASS | fresh coordinate and Phase14/15/18/20 regressions; Phase21/22 component/formal contract passes and prior source-of-truth files are unmodified |

## Findings

No blocking finding remains. The intermediate T06 component helper initially
omitted its explicit equilibrium envelope center; it was corrected before
evidence generation and did not require a model, OCP or threshold change.

The acados infeasible-envelope component intentionally emits an HPIPM minimum
step diagnostic while returning failure; the wrapper rejects it and the test
passes. This is expected negative-test output, not an accepted control result.

## Claim boundary

Phase 23 demonstrates nominal straight-reference NMPC on the current MuJoCo
simulation host. It does not validate actuator/contact identification, a real
robot, Raspberry Pi timing, roll/yaw recovery, or turning. Those require later
Phases and new evidence.
