# Phase 42 Zero-Wheel-Rate Counterfactual

For every frozen key snapshot, only native wheel velocity DOFs `[13, 8]` are set to zero. The
production Phase27 Minimal WBC lifecycle is replayed through the target tick and recomputes torque;
actual-state torque replay error is zero. No qpos, non-wheel qvel, request, contact setting or model
parameter changes.

| Tick | max torque change (Nm) | delta ddxi L/R (m/s²) | delta normal load L/R (N) |
| ---: | ---: | ---: | ---: |
| 0 | 0 | 0 / 0 | 0 / 0 |
| 1 | 2.09e-8 | +0.00190 / +0.01672 | -0.00011 / -0.00170 |
| 46 | 2.33e-6 | +0.50445 / -0.51786 | -0.05379 / +0.05631 |
| 74 | 6.48e-6 | +0.13596 / -0.99194 | -0.01778 / +0.10441 |
| 101 | 5.96e-5 | -6.53372 / -7.15582 | +0.65489 / +0.70058 |
| 110 | 0.00431 | -13.71247 / -14.07469 | +1.41770 / +1.08349 |

The rate intervention is initially immaterial but becomes acceleration-material by tick46 and
load-material by tick101. Its maximum effects before loss are `14.07469 m/s²` and `1.41770 N`.
Because this is a fixed-state local intervention, it proves hidden-rate sensitivity but is not
claimed to be a reachable rollout or a repair.

