# Phase 04: MuJoCo 基础模型与 Adapter — RECORD

Status: `complete`

> 本文件在 [`REVIEW.md`](REVIEW.md) 最终结论 PASS 后创建。

## Outcome

已交付复用 Phase 03 canonical types/messages 的 MuJoCo 3.7.0 Adapter 与 ROS2 headless runner。固定基座零输出闭环和浮动基座有界状态/reset sanity 均通过；本 Phase 证明接口、映射和安全行为，不证明站立控制或物理参数准确性。

## Delivered

- ROS 无关 model binding/state/contact/torque/reset library 与单元测试：[`wheel_leg_mujoco`](../../../../ros_ws/src/wheel_leg_mujoco/README.md)
- ROS2 runner、fixed/floating configs 和 zero-loop launch。
- 六个命名 unit-gear actuator、命名 wheel collision geoms/equalities，以及修正后的 compiled gravity。
- 双时钟与显式 reset 修订：[`robot_state_torque_command.md`](../../../interfaces/robot_state_torque_command.md)
- 工具链/编译模型 grounding：[grounding](evidence/mujoco_grounding.md) 和 [manifest](evidence/automated/mujoco_runtime_manifest.json)
- 六关节模型 zero offset：[calibration evidence](evidence/joint_offset_calibration.md)

## Verification Evidence

- [`2026-08-25-validation.md`](evidence/automated/2026-08-25-validation.md)：exact-version C++/Python load、coordinate regression、Jazzy build/tests、fixed/floating ROS runs、reset、rates and hashes.
- [`REVIEW.md`](REVIEW.md)：T01–T08 PASS；审查发现的 stale-command fail-safe 缺口已修复并回归通过；无 blocking finding。

## Decisions Confirmed

- MuJoCo C++/Python 固定为 3.7.0，physics timestep 为 2 ms；ROS 默认每五步发布一次状态。
- `sample_time_ns` 是 source-system monotonic time；Core only checks source order/`dt`. Adapter uses host steady receipt time for watchdog. This supersedes the Phase 03 host-clock interpretation without changing message schema.
- Reset handshake is ordered: `reset_simulation` first, then `reset_controller`. Source rollback is never accepted implicitly.
- Canonical joint mapping remains `q_C=-q_M+b`, `dq_C=-dq_M`, `tau_M=-tau_C`; six model offsets are frozen in canonical order and raw qpos is not published directly.
- Base pose/twist comes from torso `base_control_frame`; twist is site-Jacobian derived and world expressed.
- Only named left/right wheel collision geom against named `floor` contributes to contact.
- Actuator gears are all +1; torque sign is visible in Adapter. Command path defaults disabled and invalid, missing, stale, future, duplicate, receipt-timeout or reset-old input writes zero.
- Fixed/floating mode explicitly controls only `base_weld`; the CAD model is not duplicated.

## Deviations from PLAN

- Fixed and floating operation use one model plus an explicit reset-time `base_weld` selection and parameter files, rather than duplicated scene XML. This preserves one source model while keeping mode selection visible and testable.
- Reset is a documented two-service handshake instead of a combined orchestration service. Simulation-first ordering guarantees immediate ctrl zero, and source-time rejection closes the intermediate epoch window.
- MuJoCo's operational state always has evaluated contacts; unavailable model/data aborts publication rather than emitting a synthetic RobotState with `unknown` contacts.
- Simulink and imported CAD leg lengths differ, so absolute wheel endpoint fitting was rejected as an offset calibration method. Offsets use segment orientations and a second-pose geometric regression; model geometry fidelity remains later work.

## Known Limitations and Follow-ups

- No Controller algorithm beyond the Phase 03 exact-zero skeleton is included.
- No real actuator scale/bias/deadzone/friction/inertia calibration; Phase 05 owns those experiments.
- No hardware encoder zero, IMU installation, filter, delay or production transport validation; Phase 06 owns them.
- No claim that imported mass/inertia/contact parameters reproduce the real robot; Phase 07/08 must establish that evidence.
- ROS wall pacing is not hard realtime. Floating mode falls to the ground under zero torque and is not a standing demo.

## ROADMAP Update

- 本 Phase 对应阶段：ROADMAP Phase 04
- 状态变化：`planned → active → review → complete`
- 2026-08-25 路线修订后，下一执行项为 [Phase 14 MuJoCo 运动学与内部动力学验证](../14-mujoco-internal-dynamics-validation/PLAN.md)；该 Phase 随后已完成并 PASS。
- Phase 05 执行器辨识曾在 Phase 14 REVIEW PASS 前为 `blocked`；Phase 14 完成后已按工作流恢复，后续真机工作仍须满足各自安全 gate。

## Key Links

- [PLAN](PLAN.md)
- [REVIEW](REVIEW.md)
- [ROADMAP](../../ROADMAP.md)
