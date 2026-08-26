# Phase 19 v3 sampled-leg attribution and nonlinear pre-freeze

Result: `PASS — ALLOW_CORE_IMPLEMENTATION`

## Commands

```bash
./.venv/bin/python -m py_compile tools/experiments/run_mujoco_planar_prefreeze_v3.py
./.venv/bin/python tools/experiments/run_mujoco_planar_prefreeze_v3.py
./.venv/bin/python tools/experiments/run_mujoco_planar_prefreeze_v3.py \
  --output-dir docs/workflow/phases/19-nominal-planar-simple-standing/evidence/exploratory/2026-08-26-planar-prefreeze-v3-replay
```

Primary and fresh-process replay `summary.json` SHA-256 are both
`7148e46aa2547342d8c02c81488a2568ba310c45264df94124ff96fe06e97214`.

## Attribution

- The rejected Phase 17 fixed-base gains `Kp=12, Kd=1.5` are unstable when the leg command is sampled at 10 ms and held for five 2 ms contact-plant steps: nominal exceeds `0.6 rad`, loses bilateral contact and saturates both leg and wheel torque.
- The standing-specific candidate `Kp=8, Kd=1` preserves the frozen `2 ms / 10 ms / 5-step ZOH` contract. It is a new Phase 19 profile and does not replace the Phase 17 fixed-base profile.
- Direct `26×26` generalized-coordinate finite differences are not a release oracle. Their spectral radius varies from `1.1102` to `10.0081` with step size because independent qpos/qvel perturbations leave the compliant equality/contact constraint manifold. The sweep remains in `summary.json` as a diagnostic and is not used to claim a physical pole.

## Release evidence

- The physically admissible reset-local four-state model has rank `4`, closed-loop spectral radius `0.9847891283`, affine drift below `3.4e-13`, and maximum A/B change `7.05e-8` over `0.5×/1×/2×` central-difference steps.
- Nominal, `±0.01 rad` pitch and `±0.01 m/s` rolling cases each completed `10 s` on the non-resetting full nonlinear planar plant.
- Every case retained bilateral wheel contact for `100%` of ticks. Across the matrix: max pitch `0.009954 rad`, max X error `0.004156 m`, max height error `0.001277 m`, max active-leg error `0.01334 rad`, max wheel torque `0.755 N·m`, and max leg torque `4.831 N·m`.
- All final-state, finite, contact, position, posture and torque gates passed. This closes only the Phase 19 pre-freeze controller gate; it is not the C++ formal result and is not evidence for 3D or hardware standing.
