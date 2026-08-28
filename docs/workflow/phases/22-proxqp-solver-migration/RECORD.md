# Phase 22: ProxQP solver migration — RECORD

Status: `complete`

## Outcome

Production `wheel_leg::DenseQpSolver` now uses ProxSuite ProxQP v0.7.3 dense
`PrimalDualLDLT` behind the existing project-owned bound-form interface. The
Phase 21 12-DoF, 42-variable/104-row Weighted WBC mathematics, canonical
`RobotState -> TorqueCommand` boundary, plant, tasks and timing were unchanged.

## Delivered

- exact-version CMake/ament dependency on `proxsuite::proxsuite`, with source
  and installed-package provenance and explicit missing-dependency failure
- bound-form adapter with exact 12-equality/92-inequality WBC split, ProxQP
  status/residual mapping, compatible warm update, cold reset and fail-zero
- component/failure tests and production-adapter cold/repeated/dynamic 1000-run
  benchmark against the unchanged Phase 21 32-case corpus and oracle
- versioned solver/formal profiles, solver-block replacement inheritance,
  unambiguous manifest identity and append-only v2 authority
- 19 normal/perturbation plus 6 fault full formal, fresh replay,
  non-overwrite/hash audit and Phase14/15/18/20 compatibility regressions

## Key Results

- ROS: `24 tests, 0 errors, 0 failures`
- benchmark: cold/repeated-warm/dynamic maximum
  `0.929801/0.545467/0.689053 ms`
- oracle: physical torque error `2.8314025e-5 N m`, objective gap
  `2.4415758e-9`, stationarity `2.9134814e-9`
- formal-v2: 19/19 normal and 6/6 fault PASS; maximum Core step
  `0.978864 ms`; hard/primal/dual/stationarity
  `1.631e-8/1.631e-8/9.050e-9/9.050e-9`
- replay: 25 plant CSVs byte-exact; control differs only in wall-clock;
  142 combined manifest hash entries match current inputs/outputs
- non-overwrite and Phase14/15/18/20 fresh regressions: PASS

## Decisions

- solver identity is ProxSuite v0.7.3 commit
  `b93d7778ffc3299d84b5cb0851022a29bf24a596`, dense
  `PrimalDualLDLT`, `eps_abs=eps_rel=1e-8`, `max_iter=10000`
- only exact `lower[i] == upper[i]` rows are equalities; classification order is
  stable and the authoritative WBC split is 12/92
- warm reuse requires a previous successful solve and an identical equality
  mask/order; reset or incompatibility rebuilds cold
- non-success statuses never expose a candidate; Core keeps its existing
  fail-zero/latch/reset and independent hard/torque validation
- old ADMM settings and implementation are removed from production; Phase 21
  formal evidence remains unchanged historical WBC authority
- formal-v2 and replay-v2 are final Phase 22 authority; v1 directories are
  retained as superseded audit history

## Evidence

- [Review](REVIEW.md)
- [Dependency and profile](evidence/dependency_and_solver_profile.md)
- [Production benchmark](evidence/automated/2026-08-28-solver-benchmark-v1/README.md)
- [Formal-v2 audit](evidence/automated/2026-08-28-formal-v2/README.md)
- [Formal-v2 summary](evidence/automated/2026-08-28-formal-v2/summary.json)
- [Formal-v2 manifest](evidence/automated/2026-08-28-formal-v2/manifest.json)
- [Fresh replay-v2](evidence/automated/2026-08-28-formal-v2-replay/README.md)
- [Validation method](../../../experiments/mujoco_weighted_wbc_proxqp_validation.md)

## Reproduction

```bash
cd /home/t/W_L_ws/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to wheel_leg_mujoco \
  --cmake-clean-cache --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco
colcon test-result --verbose
cd ..
./.venv/bin/python tools/experiments/run_mujoco_weighted_wbc_formal.py \
  --config simulation/mujoco/config/phase22_proxqp_formal_v1.json \
  --output-dir docs/workflow/phases/22-proxqp-solver-migration/evidence/automated/<new-run-id>
```

The output directory must be absent or empty. Any solver/profile/source change
requires a new run namespace and complete component/formal/replay gates.

## Limits and Next Use

This record is simulation-only. It does not prove NMPC, identified/new-CAD
profiles, real-machine behavior, target-hardware determinism or hard real-time
performance. A future Phase 23 may use this ProxQP-backed WBC as its frozen
downstream layer, but must independently define and validate the NMPC model,
states, inputs, constraints and evidence.
