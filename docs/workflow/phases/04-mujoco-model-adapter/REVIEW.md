# Phase 04: MuJoCo 基础模型与 Adapter — REVIEW

Status: `review`

## Review Scope

- PLAN: [`PLAN.md`](PLAN.md)
- 审查的提交/工作树：2026-08-25 Phase 04 uncommitted worktree
- 审查者与日期：Codex, 2026-08-25

## Implementation Check

| PLAN Task | Delivered | Evidence | Result |
| --- | --- | --- | --- |
| T01 | MuJoCo 3.7.0 toolchain and compiled object/address grounding | [`mujoco_grounding.md`](evidence/mujoco_grounding.md), [manifest](evidence/automated/mujoco_runtime_manifest.json) | PASS |
| T02 | Source/receipt dual clocks and explicit two-service reset lifecycle | [interface contract](../../../interfaces/robot_state_torque_command.md), Core/wrapper tests | PASS |
| T03 | Six joint offsets and independent second-pose geometry regression | [`joint_offset_calibration.md`](evidence/joint_offset_calibration.md) | PASS |
| T04 | Named equalities/contact geoms and six unit-gear actuators | MJCF, manifest and model invariant test | PASS |
| T05 | ROS-independent Adapter state/contact/torque/reset mapping | [`adapter.cpp`](../../../../ros_ws/src/wheel_leg_mujoco/src/adapter.cpp), six Adapter tests | PASS |
| T06 | 500 Hz physics / 100 Hz state ROS runner, configs, reset and launch | [`wheel_leg_mujoco`](../../../../ros_ws/src/wheel_leg_mujoco/README.md) | PASS |
| T07 | Fixed zero-loop and floating bounded sanity | [`2026-08-25-validation.md`](evidence/automated/2026-08-25-validation.md) | PASS |
| T08 | Reproduction docs, evidence and post-change graph audit | package/workspace READMEs and validation evidence | PASS |

## Validation Results

| Validation | Actual Result | Evidence |
| --- | --- | --- |
| C++/Python model load | both exact MuJoCo 3.7.0; `nq=17`, `nv=16`, `nu=6`, gravity `[0,0,-9.81]` | [grounding](evidence/mujoco_grounding.md) |
| Phase 02 coordinate regression | PASS after actuator/naming/gravity changes | [validation](evidence/automated/2026-08-25-validation.md) |
| ROS2 Jazzy build/tests | 4-package dependency build and 13 selected tests; final whole workspace 5 packages / 18 tests, 0 failures | [validation](evidence/automated/2026-08-25-validation.md) |
| Mapping and fail-safe | offset/sign/order, one-hot torque, timeout/future/late/duplicate/NaN/reset all covered; rejected command immediately clears active command | [`test_adapter.cpp`](../../../../ros_ws/src/wheel_leg_mujoco/test/test_adapter.cpp) |
| Fixed ROS loop | finite 99.992 Hz state stream; six exact zero torques; ordered reset succeeds | [validation](evidence/automated/2026-08-25-validation.md) |
| Floating sanity | bounded finite fall/contact; deterministic unit replay and successful ROS reset | [validation](evidence/automated/2026-08-25-validation.md) |
| Post-change CBM | index refreshed; operated source paths checked; one header parser range directly read | [validation](evidence/automated/2026-08-25-validation.md) |

## Findings

### Blocking

None.

### Resolved During Review

- Initial command rejection left the previous valid command active until timeout. This violated immediate fail-zero. Review changed `acceptCommand` to clear the stored command/receipt on every rejection and added a direct ctrl-zero regression; the rebuilt test passed.

### Non-blocking

- Simulation and Controller reset are two ordered ROS services rather than one atomic service. Resetting simulation first clears ctrl immediately; any old-epoch Controller output is rejected by source-time checks until `reset_controller` runs. The sequence and rejection window are documented and observed.
- When MuJoCo contact evaluation is available, Adapter emits only `contact` or `no-contact`. If model/data are unavailable it publishes no RobotState rather than fabricating an `unknown` state; the canonical `unknown` value remains available for future Hardware Adapter use.
- The wall timer is best-effort host pacing, not realtime certification. Deterministic physics time is still fixed by the 2 ms model timestep; measured publish rate is evidence for the default timeout margins only.
- Imported mass, inertia, friction and mesh contact parameters remain nominal. No calibration, standing or dynamics-fidelity conclusion follows from this review.
- Code graph call tracing of the private runner step had low-confidence heuristic gaps. Exact source review, compiler warnings-as-errors, unit tests and real ROS execution were used for that flow instead.

## Decision and Evidence Review

- 冻结决策是否被保持：是。FLU, COM site, active quaternion, canonical joint order and Phase 02 sign/power mapping remain explicit. Core still has no MuJoCo/ROS dependency.
- 证据是否足以支持技术结论：是，但结论严格限制为基础 model load、Adapter mapping/fail-safe、reset 和 bounded fixed/floating execution。
- 是否存在需要新 Phase 的开放问题：真实 encoder/IMU/time mapping 属于 Phase 06；geometry/mass/inertia/dynamics fidelity 属于 Phase 07/08；control effectiveness belongs to Phase 09 onward.

## Verdict

`PASS`
