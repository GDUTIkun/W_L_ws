# MuJoCo nominal Weighted WBC ProxQP migration validation

## Purpose and claim boundary

This method validates only the Phase 22 solver migration on the current nominal
MuJoCo plant. It keeps the Phase 21 42-variable/104-row Weighted WBC problem,
canonical `RobotState -> TorqueCommand` boundary, 2 ms physics, 10 ms control,
5-step ZOH, plant, references, weights, case matrix and gates unchanged. A PASS
does not validate NMPC, real hardware, an identified plant or target-CPU real time.

## Frozen inputs

- formal overlay: `simulation/mujoco/config/phase22_proxqp_formal_v1.json`
- inherited authority: `simulation/mujoco/config/phase21_weighted_wbc_formal_v1.json`
- solver profile: `simulation/mujoco/config/phase22_proxqp_solver_v1.json`
- solver: ProxSuite ProxQP v0.7.3 commit
  `b93d7778ffc3299d84b5cb0851022a29bf24a596`, dense
  `PrimalDualLDLT`, 12 equality and 92 inequality rows
- residual schema: project-side hard/bound audit; adapter-side infinity norm of
  `H*x+g+Aeq^T*y+C^T*z`; ProxQP primal/dual residuals retained for cross-checking

The overlay inherits the exact Phase 21 19 normal/perturbation cases, six fault
cases and all numerical gates. The formal manifest records both files in
`config_chain` and hashes the solver profile and solver implementation.

## Preconditions and execution

Run only after component/failure tests, the unchanged 32-case oracle audit and
the 1000-run cold/repeated-warm/dynamic-warm benchmark pass.

```bash
cd /home/t/W_L_ws
./.venv/bin/python -c "import mujoco, numpy, scipy; print(mujoco.__version__, numpy.__version__, scipy.__version__)"
./.venv/bin/python -m py_compile tools/experiments/run_mujoco_weighted_wbc_formal.py
cd ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to wheel_leg_mujoco \
  --cmake-clean-cache --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco
colcon test-result --verbose
cd ..
./.venv/bin/python tools/experiments/run_mujoco_weighted_wbc_formal.py \
  --config simulation/mujoco/config/phase22_proxqp_formal_v1.json \
  --output-dir docs/workflow/phases/22-proxqp-solver-migration/evidence/automated/<run-id>
```

The output directory must not exist or must be empty before the run. Fresh
replay writes another new directory. Failures are preserved and superseded by a
new run; Phase 21 configs, manifests and outputs are never modified.

## Acceptance

All inherited per-tick solver/task/deadline, state/contact/slip/closure and
plant gates must pass for 19/19 normal cases and 6/6 fault cases. Faults must
fail-zero and latch from injection until reset; reset must restore a cold solve
and exact episode replay. Primary and fresh replay summaries must agree except
for declared wall-clock fields. Phase 14/15/18/20 regressions must also pass.
