# Phase 14 Automated Validation — 2026-08-25

## Environment and frozen inputs

- MuJoCo: `3.7.0`
- Full fixture: `simulation/mujoco/model/phase14_contact_free.xml`
- Single-leg fixture: `simulation/mujoco/model/phase14_single_leg.xml`
- Config SHA-256: `170cc15a98db5e69dae2f8be105b50d74d1f20ef3f76d8103d287c17711f4db2`
- Seed: `1404`; timestep: `0.002 s`; formal run used no hardware.

## Commands and actual results

```bash
./.venv/bin/python tools/maintenance/test_mujoco_coordinate_contract.py
./.venv/bin/python tools/experiments/run_mujoco_internal_dynamics.py
cd ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to wheel_leg_mujoco
colcon test --packages-select wheel_leg_mujoco --event-handlers console_direct+
colcon test-result --verbose
```

Actual result: coordinate contract PASS; Phase 14 nine validation groups PASS; ROS build PASS; `18 tests, 0 errors, 0 failures, 0 skipped` in the selected package dependency result set.

## Worst-case metrics

| Check | Actual worst case | Frozen limit | Result |
| --- | ---: | ---: | --- |
| FK position | `1.11e-16 m` | `1e-10 m` | PASS |
| Rotation matrix | `5.55e-16` | `1e-10` | PASS |
| Jacobian | `4.44e-16` | `2e-9` | PASS |
| Gravity generalized force | `5.18e-9 Nm` | `2e-5 Nm` | PASS |
| Static acceleration | `0` | `1e-8` | PASS |
| Mass symmetry | `0` | `1e-12` | PASS |
| Full mass minimum eigenvalue | `1.4067e-4` | `>=1e-5` | PASS |
| Full mass condition number | `46288.03` | `<=100000` | PASS |
| Forward/inverse acceleration | `1.29e-13` | `2e-8` | PASS |
| Mass-equation residual | `1.42e-14` | `2e-8` | PASS |
| Closed-chain position residual | `1.11e-16 m` | `2e-5 m` | PASS |
| Closed-chain velocity residual | `1.14e-15 m/s` | `2e-4 m/s` | PASS |
| Coupling reciprocity | `1.78e-15 rad/s²` | `2e-9 rad/s²` | PASS |
| Energy relative balance | `0.01530` | `0.03` | PASS |
| Replay determinism | `0` | `0` | PASS |
| Replay max `|q|/|dq|/|qdd|` | `0.477 / 1.408 / 4.755` | `3 / 20 / 500` | PASS |

The full mass matrix is 16×16. The complete fixed-base/two-closure constraint Jacobian has rank 10 and produces a 6-dimensional nullspace; its projected mass matrix remains positive definite. The single-leg native `+1 Nm` coupling matrix has positive diagonal entries and nonzero, reciprocal off-diagonal coupling. Detailed values and worst samples are in [`phase14_validation.json`](phase14_validation.json).

## Evidence artifacts

- [`phase14_validation.json`](phase14_validation.json): machine-readable checks and hashes.
- [`parameter_manifest.json`](parameter_manifest.json): compiled parameter provenance/status.
- [`open_loop_replay.csv`](open_loop_replay.csv): 250 deterministic samples.

## Interpretation limit

PASS means `MuJoCo internally consistent` under the frozen nominal fixtures. It does not mean `MuJoCo matches the real robot`; no new real-machine evidence was produced or used.
