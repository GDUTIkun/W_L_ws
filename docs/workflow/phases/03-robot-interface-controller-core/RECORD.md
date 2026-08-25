# Phase 03: 统一 Robot 接口与 Controller Core 骨架 — RECORD

Status: `complete`

> 本文件在 [`REVIEW.md`](REVIEW.md) 最终结论 PASS 后创建。

## Outcome

已冻结并实现跨 MuJoCo/真机 Adapter 共用的 canonical RobotState/TorqueCommand 边界、ROS 无关 C++ Core 安全骨架、ROS2 聚合消息/转换和最小 wrapper，并在 Ubuntu ROS2 Jazzy 中完成真实构建与测试。

## Delivered

- 精确字段、frame/origin、单位、时间和生命周期契约：[`docs/interfaces/robot_state_torque_command.md`](../../../interfaces/robot_state_torque_command.md)
- Simulink/ROS grounding：[`evidence/interface_grounding.md`](evidence/interface_grounding.md)
- ROS 无关类型、validation、legacy translational fixture 和 zero-output Core：[`wheel_leg_core`](../../../../ros_ws/src/wheel_leg_core/README.md)
- 聚合消息：[`wheel_leg_msgs`](../../../../ros_ws/src/wheel_leg_msgs/README.md)
- 命名转换与最小 wrapper：[`wheel_leg_ros`](../../../../ros_ws/src/wheel_leg_ros/README.md)
- 更新后的 workspace 入口：[`ros_ws/README.md`](../../../../ros_ws/README.md)

## Verification Evidence

- [`2026-08-25-validation.md`](evidence/automated/2026-08-25-validation.md)：独立 CTest 1/1；ROS2 workspace 4 packages；11 tests、0 failure；pub/sub 和依赖边界 PASS。
- [`REVIEW.md`](REVIEW.md)：全部 T01–T09 PASS，无 blocking finding。

## Decisions Confirmed

- RobotState 是 torso-COM canonical estimate：FLU base pose/twist、六关节 q/dq、左右三态 contact；不携带 raw sensor/driver diagnostics。
- `sample_time_ns` 是映射到 Controller host steady clock 的单调时间；transport sequence 和 ROS wall/header time不进入 Core physical type。
- Core API 为 `configure/reset/step(state, now_ns)`；只推进 accepted sample history，首样本 `dt=0`，其后由相邻 accepted timestamps 得到 `dt`。
- 当前 Core 对有效样本仅产生六路有限零力矩；无效、未来、过期和非单调样本被拒绝。
- package direction 为 `wheel_leg_core + wheel_leg_msgs → wheel_leg_ros`; Core 不依赖 ROS/MuJoCo/transport。
- ROS quaternion `[x,y,z,w]` 只在 named conversions 与 Core `[w,x,y,z]` 重排。

## Deviations from PLAN

- PLAN 创建时记录的工具环境为“无 ROS2 的 Windows”；执行环境实际为 Ubuntu 24.04 + ROS2 Jazzy，因此 DG06 在本 Phase 内直接以真实 `colcon build/test` 关闭，无需延期。
- 包名冻结为 `wheel_leg_core`, `wheel_leg_msgs`, `wheel_leg_ros`；未继承已缺失的 `wheel_leg_bridge`。

## Known Limitations and Follow-ups

- 本 Phase 不含 Planner/NMPC/WBC/PD/重力补偿，零力矩骨架不证明控制效果。
- MuJoCo Adapter、joint zero offset 和模型接入留给 Phase 04；Hardware Adapter 正式状态/时间/IMU验证留给 Phase 06。
- Phase 05 experimental `NormalState/NormalCommand` remains separate and is not automatically compatible.
- Existing Phase 05 source mtimes trigger non-blocking clock-skew warnings in this environment; its build/tests passed.

## ROADMAP Update

- 本 Phase 对应阶段：ROADMAP Phase 03
- 状态变化：`review → complete`
- 下一建议 Phase：Phase 04 MuJoCo 基础模型与 Adapter

## Key Links

- [PLAN](PLAN.md)
- [REVIEW](REVIEW.md)
- [ROADMAP](../../ROADMAP.md)
