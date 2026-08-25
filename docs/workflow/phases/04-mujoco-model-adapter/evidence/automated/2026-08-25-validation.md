# Phase 04 Automated Validation — 2026-08-25

Environment: Linux 6.17.0-35-generic, ROS 2 Jazzy, GCC 13.3.0, CMake 3.28.3, MuJoCo C++/Python 3.7.0.

## Model and coordinate checks

```text
.venv/bin/python tools/maintenance/test_mujoco_coordinate_contract.py
MuJoCo coordinate contract: PASS
```

```text
.venv/bin/python tools/maintenance/audit_mujoco_runtime.py \
  --scene simulation/mujoco/model/scence.xml \
  --output docs/workflow/phases/04-mujoco-model-adapter/evidence/automated/mujoco_runtime_manifest.json
MuJoCo 3.7.0: nq=17, nv=16, nu=6, nsensordata=26
Compiled gravity=[0.0, 0.0, -9.81]
Runtime frame audit: PASS
```

`/opt/mujoco-3.7.0/bin/compile` loaded and compiled the same scene. A 10,000-step, one-thread, zero-noise `testspeed` run reported 5,760 steps/s and 11.52× realtime. This is only a bounded headless load/capacity check.

## Build and unit/integration tests

From `ros_ws`:

```text
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to wheel_leg_mujoco
Summary: 4 packages finished

source install/setup.bash
colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco
colcon test-result --test-result-base build/<selected-package> --verbose
wheel_leg_core: 1 test; wheel_leg_ros: 5 tests; wheel_leg_mujoco: 7 tests
All selected results: 13 tests, 0 errors, 0 failures, 0 skipped
```

A final whole-workspace build/test, including the independent Phase 05 STM32 bridge, also completed: 5 packages and 18 tests, 0 errors/failures/skips. The pre-existing Phase 05 future-mtime clock-skew warnings remained non-fatal.

The six Adapter tests cover named model invariants and offsets, second-pose geometry, world/COM site twist against finite difference, command sign/watchdogs/fail-zero, named wheel-floor contact aggregation, and deterministic floating reset replay. Core and wrapper tests cover source-time monotonicity, explicit controller reset, conversions and ROS pub/sub.

## ROS fixed-base zero-loop

Launch:

```text
ros2 launch wheel_leg_mujoco zero_loop.launch.py floating_base:=false
```

- `/robot_state`: average 99.992 Hz over 203 samples (min 9 ms, max 11 ms); physics remains a 2 ms wall timer and publishes every five steps.
- A sampled state at simulation time 16.38 s had finite COM pose/twist, six q/dq entries and contact `[contact, contact]`.
- `/torque_command` contained six exact zeros.
- Ordered `reset_simulation` then `reset_controller` returned success; the next sampled command used the restarted source epoch and remained six exact zeros.
- During the intentional interval between reset services, old-history rejection was throttled to one warning per second; no rollback was silently accepted.

## ROS floating-base sanity

Launch with `floating_base:=true` produced finite state after free fall/contact (sample at 4.61 s), two evaluated contact values, and six exact zero torques. Ordered reset succeeded and the restarted source epoch again produced finite state and exact zero torque. Standing, stable posture and dynamics fidelity are deliberately not claimed.

## Hashes

- scene: `66cede35ad67dfda852aa15b1e6f4469aa892130a5c9e13761518157aa9f6a1d`
- included model: `1d60473d3036ef7d9488636a8c33c37ac677f6eb5ff2ee744212c3f45039a00c`
- fixed config: `ce23041a563bb3624f48566131ea9d999e76b052fd7886e623a0d77d39dbafd2`
- floating config: `54d7eaafa052f2aa20283ff856b768a3af15a78a5546ea83572c91703b230086`

Result: PASS for Phase 04 interface, mapping, safety and bounded runner criteria. No parameter calibration, standing-control or real-hardware claim is supported.

## Post-change code graph

The repository was re-indexed after the structural changes (3,924 nodes / 6,649 edges). Symbol search found `Adapter`, `AdapterConfig`, `MuJoCoNode`, the six Adapter tests and the revised `ControllerCore`. Coverage checks reported no recorded issue for the operated implementation/test paths except a retained parser partial at `adapter.hpp:28`; that constructor declaration and the complete header were read directly. Call tracing for the private runner step contained low-confidence heuristic gaps, so runner-flow conclusions are based on the reviewed source plus real ROS execution rather than that trace alone.
