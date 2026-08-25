# Phase 02: 坐标系、单位、关节顺序与接口语义 — RECORD

Status: `complete`

> 本文件在 [`REVIEW.md`](REVIEW.md) 的最终结论为 PASS 后创建。

## Outcome

跨 Simulink、MuJoCo、Controller Adapter 和后续真机的 canonical world 已冻结为 FLU，并以可重复审计、方向性测试和显式后续 gate 取代隐式换轴与猜测零位。

## Delivered

- [`coordinate_frame_contract.md`](../../../models/coordinate_frame_contract.md)：canonical、legacy pack、pose/twist/acceleration/wrench、base/COM/IMU 和 joint 语义。
- [`wheel_leg.xml`](../../../../simulation/mujoco/model/wheel_leg.xml)：保留 CAD 根，新增 nominal torso-COM `base_control_frame` site 与防误用注释。
- [`joint_coordinate_mapping.md`](evidence/joint_coordinate_mapping.md)：冻结六关节相对轴/符号及 offset 标定门槛。
- [`test_coordinate_frame_contract.m`](../../../../tools/maintenance/test_coordinate_frame_contract.m) 与 [`test_mujoco_coordinate_contract.py`](../../../../tools/maintenance/test_mujoco_coordinate_contract.py)：跨系统代数和 MuJoCo 方向性回归。
- Simulink/MuJoCo 静态、运行时和人工证据：[`evidence/`](evidence/) 与 [`USER_CHECKPOINT.md`](USER_CHECKPOINT.md)。

## Verification Evidence

- MATLAB algebraic test：PASS；Simscape→FLU、legacy pack、positive yaw、wrench origin/frame round-trip。
- MuJoCo coordinate test：PASS；COM site、FLU、六 joint axes/sign、左右微扰、positive rolling、active wxyz 和 continuous yaw。
- MJCF audits：11 bodies、10 joints、6 sites、19 sensors、0 duplicate names；runtime probes PASS。
- 当前用户批准 `source.slx`：manifest 已更新；5 s smoke `simulationCompleted=true`、`controlStable=true`。
- 人工三视图：Simscape X前/Y上/Z右，MuJoCo X前/Y左/Z上；审查为相应范围 PASS。
- `git diff --check`：PASS。

## Decisions Confirmed

- Canonical `{N}`：X 前、Y 左、Z 上，右手 FLU；`g_N=[0,0,-g]`。
- Simscape `{S}`：X 前、Y 上、Z 右；`R_N_from_S=[1 0 0;0 0 -1;0 1 0]`。
- MuJoCo world `{M}` 已为 FLU，`R_N_from_M=I`。
- Controller 旧 `[前,右,上]` 只是 field pack，不是空间 frame。
- `base_body` 是 CAD frame；`base_control_frame` 位于 torso COM 并继承 body axes；真实 `imu_frame` 独立。
- 六关节 `q_C=-q_M+b_joint`、`dq_C=-dq_M`、`tau_M=-tau_C`，左右不额外镜像。
- 坐标问题可用 site + Adapter 解决，当前不需要 CAD 重导出。

## Deviations from PLAN

- 用户有意断开 `source/PD_only/6-DOF Joint` LConn2–LConn7；Codex 未恢复。当前 manifest 与 5 s smoke 已覆盖该批准变化。
- 真机 joint/IMU 核对未在本 Phase 执行；按用户决定和证据边界转 Phase 06。
- 逐 joint zero offset 无法从当前 CAD 导入姿态可靠推出，转 Phase 04 数值 matching-pose 标定。

## Known Limitations and Follow-ups

- Phase 04：修正 gravity/weld/actuator 基础模型，并用两个姿态冻结六个 `b_joint`。
- Phase 06：落地真实 IMU 安装 pose、encoder offsets、torque 正方向和 RobotState sensor semantics。
- Phase 07：用真实/验证模型标定质量与 COM；质量变化会触发 `base_control_frame` stale test。
- 当前 MuJoCo torso mass 2.588 kg 与 Simulink 简化 base mass 3.0 kg 不在本 Phase 对齐。

## ROADMAP Update

- 本 Phase 对应阶段：02 坐标系、单位、关节顺序与接口语义。
- 状态变化：`review → complete`。
- 下一建议 Phase：Phase 04 MuJoCo 基础模型与 Adapter；Phase 03 可并行冻结公共类型。

## Key Links

- [PLAN](PLAN.md)
- [REVIEW](REVIEW.md)
- [ROADMAP](../../ROADMAP.md)
