# P23-T01 grounding and reuse audit

Date: 2026-08-28

> Route update, 2026-08-28: this audit predates the user-directed acados
> revision. Its live-code, interface and historical-model findings remain
> valid, but the former prohibition on acados-generated production artifacts
> is superseded by the current [PLAN](../PLAN.md). The revised route generates
> a new solver from the approved 12D model; it does not approve or reuse the
> historical Simulink/Euler S-function.

## Live-code authority

- CBM project `W_L_ws`, full index generation `2026-08-28T07:49:08Z`,
  6248 nodes / 11177 edges.
- `ControllerCore::stepWeightedWbc` remains the only production injection
  point: it builds `WbcReference`, assigns the fixed 12D
  `interaction_wrench_flu`, calls `WeightedWbcController`, then independently
  rejects solver/model/torque failures with zero output and a latched fault.
- `NominalWbcModel::evaluate` already owns current-nominal passive
  reconstruction, 12D reduced mass/bias, actuation, contact Jacobian/bias,
  contact frames and bilateral contact-wrench-to-base-FLU transforms.
- Production C++ contains no NMPC symbol. The additive Phase 23 layer must
  therefore stop at the existing 12D wrench boundary; public
  `RobotState/TorqueCommand` and the Phase 21/22 WBC remain unchanged.

## Reuse boundary

- Reuse `nominal_wbc_profile_data.hpp`, `NominalWbcModel`, Phase 21 model/QP
  oracles, the ProxSuite v0.7.3 dependency, WBC component corpus,
  `weighted_wbc_loop` and the Phase 22 formal wrapper/schema.
- Do not enlarge the WBC fixed 42D `DenseQpSolver` for the upper problem and
  do not add another plant/Adapter/formal framework.
- `RobotState` base pose/twist is the `base_control_frame` site. Any upper
  wheel-relative coordinate must therefore be defined from that canonical
  site and current reconstructed wheel geometry, not from a historical body
  origin.

## Historical candidate conflicts

- Simulink `full_base_nmpc_state_signal` uses Euler angles/rates; production
  uses quaternion plus world-axis angular velocity.
- Historical `full_base_body_dynamics` reconstructs moments from lever arms
  and names the input a wheel/body interaction wrench. The current WBC 12D
  reference is produced by transporting each contact-centred wrench to the
  canonical base control site; frame, reference point and sign therefore
  require a new oracle rather than direct copying.
- Historical nominal values use a different wheel radius/model family and
  cannot replace the current nominal Phase 15/21 profile.
- Under the original sparse-ProxQP route, acados-generated S-functions/C code
  were reference-only. The current PLAN supersedes that route only for a newly
  generated solver derived from the approved 12D model; the historical
  Simulink/Euler artifact remains prohibited.
- `full_base_nmpc_command` holds the last valid output on failure; production
  must instead preserve Phase 22 zero/latch/reset semantics.

## Coverage and limitations

All 20 exact Core/WBC/runner/Simulink paths checked for this task report
`no_recorded_issue + metadata_match`. Bounded scope checks report only the
known parser ranges `test_dense_qp_solver.cpp:29`, `adapter.hpp:28` and
`deterministic_loop.cpp:438`, plus an excluded `__pycache__`; the three source
ranges were read directly and do not hide an NMPC path. This remains a
best-effort signal, not proof of completeness. `docs/` and `tools/` are
excluded from CBM by policy and were verified from live source and the local
Graphify graph.

## Baseline

From `/home/t/W_L_ws/ros_ws`:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to wheel_leg_mujoco \
  --cmake-clean-cache --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco
colcon test-result --verbose
```

Result: four packages built; `24 tests, 0 errors, 0 failures, 0 skipped`.
This proves only the Phase 22 baseline before NMPC work.
