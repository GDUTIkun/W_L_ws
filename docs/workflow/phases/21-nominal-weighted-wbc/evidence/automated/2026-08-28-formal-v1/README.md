# P21-T11 formal v1 run and audits

Date: 2026-08-28. All commands below were executed for real; results are as
observed on this machine.

## Formal entry

From `ros_ws/` with ROS jazzy sourced:

```text
colcon build --symlink-install --packages-up-to wheel_leg_mujoco   # 4 packages PASS
colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco
colcon test-result --verbose   # 24 tests, 0 errors, 0 failures, 0 skipped
```

## Formal matrix

```text
./.venv/bin/python tools/experiments/run_mujoco_weighted_wbc_formal.py \
  --output-dir docs/workflow/phases/21-nominal-weighted-wbc/evidence/automated/2026-08-28-formal-v1
```

Result: exit 0, `pass: true`, 19/19 normal cases and 6/6 fault cases PASS.
Worst metrics across normal cases (gate in parentheses):

| metric | worst | gate |
| --- | --- | --- |
| max core step | 9.90157 ms | ≤ 10 ms |
| max abs X / Y | 2.075e-3 / 2.006e-3 m | ≤ 0.02 m |
| max height error | 1.680e-4 m | ≤ 0.01 m |
| max roll / pitch / yaw | 6.377e-3 / 7.181e-3 / 1.977e-2 rad | ≤ 0.03/0.03/0.05 |
| max leg error | 1.480e-2 rad | ≤ 0.03 rad |
| max final linear / angular speed | 1.031e-3 m/s / 6.718e-3 rad/s | ≤ 0.02 / 0.1 |
| max hard violation | 1.070e-7 | ≤ 2e-7 |
| max primal / dual / stationarity | 1.265e-7 / 6.506e-8 / 4.205e-8 | ≤ 2e-7 |
| max normalized slack | 3.728e-3 | ≤ 0.01 |
| max task residual / cost | 5.523e-3 / 4.290e-5 | ≤ 0.02 / 0.001 |
| min wheel normal load | 31.27 N | ≥ 1 N |
| max penetration | 5.369e-4 m | ≤ 0.004 m |
| max rolling / lateral slip | 8.272e-3 / 1.731e-3 m/s | ≤ 0.05 |
| max closure residual | 1.835e-4 m | ≤ 2e-4 m |
| bilateral contact fraction | 1.0 | ≥ 1.0 |
| ZOH difference / adapter sign error | 0 / 0 | ≤ 0 |
| saturation count | 0 | 0 |

Note: the 9.90 ms worst core step is a wall-clock scheduling spike on this
machine; the replay run's worst was 7.93 ms. The deadline gate passed in both
runs but has thin headroom on loaded hosts.

## Fresh replay

Same frozen inputs into `2026-08-28-formal-v1-replay` (exit 0, all PASS).
Comparison audit:

- manifest frozen-input hashes identical (config, runner, scene, all 12
  source/profile hashes);
- all 25 plant CSVs byte-exact between the two runs;
- control CSVs differ only in the `core_step_ns` wall-clock column
  (22 389 differing cells, no other column differs);
- summary.json identical after removing the wall-clock
  `maximum_core_step_ms` metric.

## Non-overwrite

Re-running the wrapper against `2026-08-28-formal-v1` exits with code 2
(`Refusing non-empty output directory`) and leaves the 52-file listing
unchanged. The runner itself additionally refuses any existing CSV path.

## Historical regressions (fresh directories, no old evidence touched)

```text
./.venv/bin/python tools/maintenance/test_mujoco_coordinate_contract.py   # PASS, exit 0
./.venv/bin/python tools/experiments/run_mujoco_internal_dynamics.py \
  --output-dir data/experiments/2026-08-28-phase21-phase14-regression      # PASS, exit 0
./.venv/bin/python tools/experiments/run_mujoco_closed_chain_kinematics.py \
  --output-dir data/experiments/2026-08-28-phase21-phase15-regression      # PASS, exit 0
./.venv/bin/python tools/experiments/run_mujoco_contact_floating_base.py \
  --output-dir data/experiments/2026-08-28-phase21-phase18-regression/raw  # overall_pass true
./.venv/bin/python tools/experiments/run_mujoco_3d_standing_formal.py \
  --output-dir data/experiments/2026-08-28-phase21-phase20-regression      # pass true
```

`python -m py_compile` on the four regression tools PASS. `git diff --check`
PASS.

## Scope boundary

This closes the P21-T11 execution deliverables: frozen-input formal, fresh
replay, non-overwrite and Phase 14/15/18/20 regressions. It does not close
DG21-07 or DG21-08; verdict authority remains with P21-T12 REVIEW, and no
real-machine, NMPC, terrain or real-time conclusion is drawn from these
simulation-only results.
