# Phase 21 42D Weighted-Task Nonlinear Pre-Holdout Freeze

Date: 2026-08-27  
Verdict: **local, tuning and holdout gates PASS; P21-T06 done; DG21-05 CLOSED**

## Scope and freeze point

This record freezes the Phase 21 task candidate after local attribution and the declared
tuning split, before any 42D holdout case is run. It does not authorize Core integration.

- local task profile: `simulation/mujoco/config/phase21_task_prefreeze_42d.json`
- nonlinear frozen profile: `simulation/mujoco/config/phase21_task_prefreeze_42d_nonlinear_frozen_v1.json`
- nonlinear runner: `tools/experiments/validate_weighted_wbc_tasks_42d_nonlinear.py`
- competition audit: `tools/experiments/audit_weighted_wbc_tasks_42d_competition.py`
- frozen profile SHA-256: `c75ce6f5276cf4db80862125ed6b423756e2287c2cd5f8f0c525bc84b92ad50f`
- nonlinear runner SHA-256: `98670bd6c23a9d6c4dc69b6255ca19a4420188c296044bf06a8e318ee0e4c911`
- competition audit SHA-256: `72a9f7b0bfed89def061c48d561e99ff3e78e4822eeb3936a9fc247c00aa2783`

The frozen task weights and wrench-slack penalty are all `1`; scaled regularization is
`1e-6`. PD gains are `[kp,kd]`: base X `[9,6]`, height `[25,10]`, orientation
`kp=[25,25,9]`, `kd=[10,10,6]`, and hip/knee `[36,12]`. Timing remains 10 ms control,
2 ms physics and five physics steps per held command. Disturbances begin at control tick
100 and last ten ticks.

Before holdout, the normalized envelopes are frozen as:

| Gate | Frozen maximum | Tuning worst |
| --- | ---: | ---: |
| wrench slack | `0.01` | `0.003129` |
| any task residual | `0.02` | `0.004193` |
| summed task cost | `0.001` | `0.0001059` |

These provide approximately 3.2x, 4.8x and 9.4x headroom over the tuning observations.
They may not be relaxed after observing holdout.

## Local attribution and accounting

The authoritative competition result is
`data/experiments/2026-08-27-phase21-task-competition-42d-local-v3/`.
Across the frozen 32-case corpus, baseline plus six single-disabled variants all solve,
remain finite and satisfy the 104 hard rows. Enabling each declared task strictly reduces
its aggregate own normalized SSE. The complete Hessian and gradient reconstruct with zero
error from the six motion/contact/wrench tasks, the explicit slack penalty and regularizer;
the maximum residual-form objective accounting error is `1.07e-14`. The actual
wrench-fidelity slack block is `-I_12`, its target equals the generated FLU reference, and
slack remains absent from every hard row.

The earlier competition v1/v2 directories are retained as superseded implementation/audit
iterations; v3 replaces the v2 string-only slack-sign check with an actual matrix/target
check.

## Frozen tuning results

All four declared 10 s tuning cases pass under the frozen profile and runner hashes:

- `data/experiments/2026-08-27-phase21-task-frozen-42d-tuning-nominal-v1/`
- `data/experiments/2026-08-27-phase21-task-frozen-42d-tuning-pitch-positive-v1/`
- `data/experiments/2026-08-27-phase21-task-frozen-42d-tuning-roll-positive-v1/`
- `data/experiments/2026-08-27-phase21-task-frozen-42d-tuning-yaw-positive-v1/`

Across the four cases, non-finite ticks, solver failures and saturation counts are zero;
maximum hard and bound violation are `2.22e-16`; bilateral contact fraction is `1.0`.
Worst plant outcomes are `|x|=1.250 mm`, `|y|=1.815 mm`, height error `0.087 mm`,
roll `0.005783 rad`, pitch `0.006974 rad`, yaw `0.017142 rad`, penetration `0.515 mm`,
rolling/lateral slip `0.007111/0.001543 m/s`, and closure residual `0.183 mm`.

## Decision

The 42D task scales, equal weights, gains, wrench reference/slack penalty, task/slack
envelopes, tuning/holdout split and nonlinear runner are frozen for the next gate. The
next permitted action is to execute the nine already-declared holdout cases without
changing these inputs.

## Frozen holdout results

All nine declared 10 s holdout cases pass without changing the frozen profile, runner or
thresholds. The first four results were created on 2026-08-27 and the remaining five on
2026-08-28 after an orchestration quota interruption; this was an execution-environment
continuation, not a model or gate failure. Each directory contains a 1000-tick summary,
1001-line CSV and manifest with matching input, source and output hashes.

Across the nine holdouts, non-finite ticks, solver failures and saturation counts are zero;
maximum hard and bound violation are `2.22e-16`; bilateral contact fraction is `1.0` and
minimum wheel normal load is `31.015 N`. Worst outcomes are:

| Metric | Holdout worst | Frozen maximum |
| --- | ---: | ---: |
| `|x|` | `0.002079 m` | `0.02 m` |
| `|y|` | `0.001815 m` | `0.02 m` |
| height error | `9.567e-5 m` | `0.01 m` |
| roll / pitch / yaw | `0.005783 / 0.007176 / 0.019272 rad` | `0.03 / 0.03 / 0.05 rad` |
| penetration | `0.000525 m` | `0.004 m` |
| rolling / lateral slip | `0.007979 / 0.001543 m/s` | `0.05 / 0.05 m/s` |
| closure residual | `0.00018336 m` | `0.0002 m` |
| normalized slack | `0.003535` | `0.01` |
| task residual | `0.005452` | `0.02` |
| task cost | `0.0001502` | `0.001` |

The nine authoritative result directories match
`data/experiments/2026-08-{27,28}-phase21-task-frozen-42d-holdout-*/`, with case identity
and split checked from each summary rather than inferred from directory spelling.

## Final decision

The frozen 42D task set, normalization, equal weights, PD gains, equilibrium FLU wrench,
slack sign/penalty/envelope, competition accounting and 10 s tuning/holdout gates are
accepted. P21-T06 is done and DG21-05 is closed. This authorizes P21-T07 to implement the
same frozen mathematics; it does not itself prove production C++ parity, runtime safety,
formal replay or Phase 21 completion.
