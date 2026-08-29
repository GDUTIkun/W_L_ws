# Phase 27 Review

Verdict: `PASS`

Blocking findings: 0

## Reviewed scope

The review covers current-nominal MuJoCo-only wheel-state/planner recovery,
wheel-to-body internal interaction-wrench closure, the 16-state relative
rotation-vector model, checked-in acados v2 artifact, retained 2/10/20 ms
schedule, opt-in Minimal 42D WBC, runtime/fault/logging integration and frozen
T0--T3 formal attribution. It excludes identified plant, real hardware,
terrain and any add-back stabilization task.

## Gate review

| Gate | Result | Evidence |
| --- | --- | --- |
| Wheel state/planner | PASS | MuJoCo geometry/FD oracle, common/differential signs, workspace, governor and reset |
| Interaction wrench | PASS | Newton--Euler/action-reaction/transport/virtual-work oracle and affine 42D reconstruction |
| 16-state model | PASS | two 10 ms RK4 substeps; continuous/next/Jacobian/current-parameter oracle |
| Schedule | PASS | 2/10/20 retained after v2 comparison; 1/5/20 closed no new gate |
| acados component | PASS | deterministic v2 generation; next/A/B, bounds, defect, stationarity, reset and deadline |
| Minimal WBC component | PASS | 42 variables/104 hard rows unchanged; only approved three soft blocks; ProxQP corpus |
| Runtime/fault | PASS | opt-in mode, age/ZOH, additive logs, four exact-zero faults, latch/reset |
| Formal method | PASS | frozen config before primary, six T0--T3 cases, synthetic evaluator oracle and manifests |
| Controller candidate | EXPECTED FAIL | T0--T2 first fail at safety envelope; T3 first fails native NMPC stationarity audit |
| Replay/non-overwrite | PASS | plant byte-exact; control exact excluding three declared clock fields; collision rejected |
| Build/regression | PASS | Release four-package build and 32 tests, zero errors/failures/skips |

## Findings

No blocking workflow or evidence finding remains. `formal-v1` is retained as
evaluator-inconclusive because of a one-tick moving-reference alignment error
and invalid interpretation of post-latch zero diagnostics. Append-only v2
corrected only evaluation; it did not change thresholds or controller inputs.

One full-suite run saw an otherwise healthy cold-start delayed to 10.98 ms by
the non-real-time host; production Core correctly latched. The unit fixture now
retries only this externally delayed cold-start condition. The production
10 ms gate, dedicated formal timing evidence and explicit late-fault test are
unchanged. The final full suite passed.

## Verdict rationale

Phase 27's goal explicitly permits a diagnosed Minimal FAIL. Every upstream
physical/component gate is closed, every formal case has one first-failure
layer, and the candidate failure is reproducible without modifying the frozen
architecture. Adding base/differential tasks or changing SQP-RTI behavior
would be new work and is correctly deferred.

## Claim boundary

This Phase proves contracts and diagnoses the current Minimal candidate on the
current nominal simulation host. It does not approve the Minimal controller
for deployment and does not prove that a particular add-back task or fast
low-level loop is necessary or sufficient.
