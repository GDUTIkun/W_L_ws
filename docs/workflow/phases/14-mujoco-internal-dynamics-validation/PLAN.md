# Phase 14: MuJoCo 运动学与内部动力学验证 — PLAN

Status: `complete`

## Goal

在不连接、不驱动、不采集真机的前提下，证明当前 MuJoCo 多刚体 plant 的运动学、约束、重力、惯量、正逆动力学、耦合、能量和确定性实现内部自洽，并形成后续 MuJoCo–真机辨识可直接复用的激励、状态和分析基准。

## Current State

- 已有：Phase 04 已交付 MuJoCo 3.7.0 模型、canonical Adapter、fixed/floating runner、六路 unit-gear actuator、joint offset、COM-site twist、wheel-floor contact 聚合及零力矩安全闭环。
- 已有：总迁移路线已经规定“先完成 MuJoCo 运动学与单腿动力学内部验证，再进入 MuJoCo–真机共同辨识”。
- 已有：Phase 02/04 的坐标、关节顺序、符号、第二姿态几何、site Jacobian 和模型编译清单可作为回归基线。
- 缺少：覆盖工作空间的 FK/Jacobian 数值验证、单腿/双腿测试边界、参数来源清单、重力广义力、质量矩阵、正逆动力学、闭链约束、耦合、能量与开环回放证据。
- 路线纠正：已有 Phase 05 是稳定编号的真机执行器辨识 Phase，不改名复用；本 Phase 虽编号为 14，但在 ROADMAP 执行顺序上位于 Phase 05 之前。

## Scope

- Ground 编译模型的刚体拓扑、自由度、闭链 equality、actuator、质量/COM/inertia、关节 passive 参数、接触参数和参数来源，区分 imported/nominal/derived/unknown。
- 冻结 MuJoCo-only 测试 plant：至少包含 fixed-base、无轮地接触的单腿动力学夹具；必要时保留完整双腿闭链模型作为约束回归，但不得用临时不可追溯的运行时改模冒充正式 scene。
- 在零位、典型姿态、工作空间边界附近和确定性采样姿态验证 hip/knee/wheel center/contact site 的 FK、姿态、Jacobian、速度预测和 wheel rolling direction。
- 验证多个姿态下的重力方向、重力广义力和静态平衡关系。
- 验证质量矩阵的维度、对称性、正定性、条件数和姿态依赖，并明确 constrained/unconstrained 坐标口径。
- 验证正逆动力学一致性、闭链 constraint residual、单关节激励下的惯性耦合方向和数量级。
- 建立无接触、受限输入的开环回放，记录 q/dq/qdd、广义力、约束残差和机械能/功率收支；同 seed/初值/reset 必须确定性重放。
- 输出可供 Phase 05/07/08 复用的场景、激励、日志 schema、分析脚本、容差和基线数据。

## Out of Scope

- 任何真机上电、关节转动、Load Cell、encoder/IMU 采集、STM32 板级联调或硬件参数结论。
- 根据真机数据拟合 torque scale/bias、摩擦、反射惯量、mass/COM/inertia、延迟或接触参数。
- Joint PD、重力补偿、站立、WBC、NMPC 或以闭环稳定性代替 plant 自洽验证。
- 声称 imported CAD/nominal 参数已经准确描述真机。
- 轮地接触参数保真度；本 Phase 只保留必要的 contact-free 动力学边界及已有 contact 语义回归。

## Frozen Decisions

- 本 Phase 为严格 MuJoCo-only gate；REVIEW PASS 前，Phase 05 的真机 pilot/正式辨识和 Phase 06 的真机传感器验证不得开始或恢复。
- MuJoCo 精确版本保持 3.7.0，canonical FLU、COM `base_control_frame`、active `[w,x,y,z]`、六关节顺序和 `q_C=-q_M+b`、`dq_C=-dq_M`、`tau_M=-tau_C` 不变。
- 内部自洽与真机准确性分开：本 Phase 可在 nominal 参数下 PASS，但每个参数必须标来源和待辨识状态；PASS 不能升级成真机一致性结论。
- Dynamics checks 必须直接读取 MuJoCo 编译模型和真实计算结果，不从 XML 外观、成功构建或零输出闭环推断。
- 测试输入、初值、约束模式、solver、timestep、seed、采样点和容差必须在运行前冻结并进入证据；不得看到结果后放宽阈值。
- Simulink 只用于两者物理假设重合部分的回归；已知简化模型差异不得通过退化 MuJoCo 多刚体模型来消除。
- Phase 05 的现有实验固件/ROS bridge 代码保持原样冻结；本 Phase 不修改或运行真机链路。

## Open Questions / Decision Gates

- **DG01 / CLOSED / CODEX_DECISION — 正式 test plant：** 冻结完整双腿 contact-free fixture 与固定基座五刚体闭链单腿 fixture；开链子测试显式关闭 closure，边界由名字和编译惯性 invariant 自动检查。
- **DG02 / CLOSED / EVIDENCE — 参数 provenance：** `model_parameter_grounding.md` 与 `parameter_manifest.json` 已区分 imported nominal、compiled derived、project nominal 和 unknown hardware value。
- **DG03 / CLOSED / CODEX_DECISION — 参考运动学：** 使用独立齐次变换与解析叉乘 Jacobian，覆盖 7 个冻结姿态并与 MuJoCo body/Jacobian 输出比较。
- **DG04 / CLOSED / EVIDENCE — dynamics 容差：** 采样、seed、solver、激励与所有阈值已在 `phase14_validation.json` 正式运行前写入版本化 config；正式结果没有通过放宽阈值获得。
- **DG05 / CLOSED / EVIDENCE — Gate B：** 九项 MuJoCo-only gate、Phase 04 坐标和 Adapter 回归全部 PASS；Phase 05 的 MuJoCo 前置 blocker 已解除，但真机工作仍受其自身 DG01–DG06 与安全 gate 约束。

## Interfaces and Compatibility

- 输入：版本化 MuJoCo scene/config、确定性 q/dq/qdd/torque 激励、seed、solver/timestep 和 canonical joint mapping。
- 输出：机器可读 parameter manifest、pose/Jacobian/dynamics/energy sweep、开环轨迹、质量报告和 REVIEW evidence。
- 必须保持：Phase 02–04 interface/schema、Adapter mapping、模型资产、Phase 05 实验代码默认行为。
- 允许改变：新增 MuJoCo-only fixture scene、测试/分析工具、非生产 config 和 evidence；若发现 Phase 04 model bug，必须先记录 finding 并回归 Phase 04 契约测试。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| T01 | Ground 当前多刚体模型与参数来源，关闭 DG02 | Phase 04 manifest、MJCF/assets、CAD/文档 | `evidence/model_parameter_grounding.md` 与机器可读 provenance manifest | 编译模型逐 body/joint/constraint/parameter 可追溯；unknown 明确列出 | done |
| T02 | 冻结单腿/完整模型测试边界，关闭 DG01 | T01、闭链拓扑、现有 scene | MuJoCo-only fixture scene/config 与 invariant tests | fixed/contact-free/actuator/constraint mode 均按名字断言；Phase 04 regression 不回退 | done |
| T03 | 建立独立 FK/Jacobian 工作空间 sweep，关闭 DG03 | T02、Phase 02/04 geometry evidence | 参考实现、采样集、结果 JSON/CSV | pose/orientation/Jacobian/finite-difference/rolling direction 在预冻结容差内 | done |
| T04 | 验证重力与静态关系 | T01–T03 | 多姿态 gravity generalized-force 与 equilibrium evidence | 重力方向、力矩符号、静态残差和姿态依赖通过 | done |
| T05 | 验证质量矩阵与数值条件 | T02/T03 | M(q) sweep、eigenvalue/condition report | 维度、对称性、正定性、有限值与预冻结条件数门槛通过 | done |
| T06 | 验证正逆动力学和闭链约束 | T02–T05 | forward/inverse/constraint residual suite | qdd/force round-trip、constraint residual 和 solver 状态通过预冻结阈值 | done |
| T07 | 验证惯性耦合与能量/功率收支 | T02–T06 | one-hot 激励、耦合矩阵和 energy audit | 耦合方向/数量级可解释；无源/受驱工况能量误差有界 | done |
| T08 | 建立确定性开环回放 | T02–T07 | 固定初值/seed/输入轨迹、日志和摘要 | bounded run 无 NaN/Inf；reset 重放逐样本满足冻结容差 | done |
| T09 | 冻结后续共同辨识复用契约 | T03–T08 | excitation/state/log schema、baseline configs 和 comparison entrypoint | Phase 05/07/08 可用同输入与分析口径，不包含真机结果 | done |
| T10 | 汇总自动证据并准备 REVIEW | 全部任务 | validation report、README、Execution Notes 和 REVIEW 输入 | DG01–DG05 全部关闭，无 hardware execution evidence | done |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Automated

- Phase 04 coordinate/runtime/build/test regression 必须继续 PASS。
- 新增 MuJoCo-only C++/Python tests 覆盖 fixture invariants、FK/Jacobian sweep、gravity、M(q)、forward/inverse、constraints、coupling、energy 和 deterministic replay。
- 所有 sweep 输出 finite-value、residual distribution、worst-case sample 和 frozen threshold；只报平均值不足以 PASS。
- 全部验证必须能在 headless 环境重复运行，不依赖 viewer 或人工拖动模型。

### Manual / Evidence

- Codex 审查 parameter provenance、最差样本、solver/constraint residual 和能量解释；不安排用户进行真机操作。
- 若测试暴露模型实现错误，Phase 保持 active/rework，先修复并回归；若只暴露 nominal 参数未知，记录为后续辨识输入而不伪造数值。
- REVIEW 必须明确区分 `MuJoCo internally consistent` 与 `MuJoCo matches real robot`；后者在本 Phase 永远不能为 PASS。

## Acceptance Criteria

- [x] T01–T10 完成，DG01–DG05 由真实 MuJoCo 计算证据关闭。
- [x] 存在正式、可重复的 fixed-base/contact-free single-leg test plant，并保持完整模型约束回归。
- [x] FK/Jacobian sweep、gravity、M(q)、forward/inverse dynamics、constraint residual、coupling 和 energy checks 全部通过冻结阈值。
- [x] 开环回放 bounded、finite、deterministic，输出包含最差样本与完整复现配置。
- [x] 全部参数有 provenance/status，不把 nominal/imported/unknown 描述为真机标定值。
- [x] 生成可供后续 MuJoCo–真机共同辨识复用的输入、状态、日志和比较入口。
- [x] 没有执行或引用新的真机运行证据来关闭本 Phase。
- [x] REVIEW 为 PASS 后才解除 Phase 05 blocker。

## Execution Notes

- 2026-08-25：因用户确认“先不动真机，先做一轮 MuJoCo 动力学验证”，创建本稳定编号 Phase。编号 05 已被执行器辨识历史占用，因此保留其含义，本 Phase 使用新编号 14，但 ROADMAP 执行顺序排在 Phase 05 前。
- 2026-08-25：本次只修正路线并制定 PLAN，未开始 MuJoCo 动力学实现或验证，状态保持 `planned`。
- 2026-08-25：状态 `planned → active`。T01/T02 ground 完整模型并冻结完整 contact-free 与固定基座五刚体闭链单腿 fixture；机器可读 manifest 证明单腿 fixture 的五个 body inertial 与完整模型编译值逐项一致。
- 2026-08-25（模型 finding）：首轮 sweep 发现 imported MJCF 的截断欧拉角令名义共轴偏离约 `3.67e-6`，闭链 Jacobian rank 由物理预期 10 变为 11。将结构角修正为精确 `pi/2`/`pi`，并从修正后几何重新计算 model-coordinate offsets；没有改变 canonical 映射公式。
- 2026-08-25（T03–T08）：运行 `./.venv/bin/python tools/experiments/run_mujoco_internal_dynamics.py`，九项 gate 全部 PASS。完整结果、最差样本、输入 hash 和 250 样本 replay 见 `evidence/automated/`。
- 2026-08-25（T09/T10）：冻结正式实验方法、复用契约、数据包 README、simulation/tools README 和参数 provenance；明确 native actuator sign 与 canonical torque sign 的转换。
- 2026-08-25：运行 coordinate contract、`colcon build --packages-up-to wheel_leg_mujoco`、六项 Adapter gtest 与依赖 test result；最终为 `18 tests, 0 errors, 0 failures, 0 skipped`。状态 `active → review`，REVIEW Verdict 为 PASS 后创建 RECORD 并转 `complete`。

## Blockers

None. 本 Phase 不依赖真机设备、Load Cell、STM32 板级通信或 Hardware Adapter。
