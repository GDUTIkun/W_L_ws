# Phase 14: MuJoCo 运动学与内部动力学验证 — RECORD

Status: `complete`

> 本文件在 [`REVIEW.md`](REVIEW.md) Verdict 为 PASS 后创建。

## Outcome

当前 MuJoCo 3.7.0 nominal plant 已在不连接真机的条件下通过运动学、重力、质量矩阵、正逆动力学、闭链、耦合、能量和确定性开环回放内部自洽验证，Gate B 关闭。

## Delivered

- 完整 contact-free 与固定基座五刚体闭链单腿 fixture，以及冻结 config：[`simulation/mujoco`](../../../../simulation/mujoco/README.md)
- 可执行正式 sweep：[`run_mujoco_internal_dynamics.py`](../../../../tools/experiments/run_mujoco_internal_dynamics.py)
- 正式方法与数据追溯：[实验方法](../../../experiments/mujoco_internal_dynamics_validation.md)、[data README](../../../../data/experiments/2026-08-25-mujoco-internal-dynamics/README.md)
- 参数来源与复用边界：[`model_parameter_grounding.md`](evidence/model_parameter_grounding.md)、[`reuse_contract.md`](evidence/reuse_contract.md)
- 自动结果、manifest 和 replay：[`evidence/automated/`](evidence/automated/)
- 修复 imported MJCF 截断结构角，并同步修正 canonical model offsets；Phase 04 Adapter 回归保持 PASS。

## Verification Evidence

- Phase 14 sweep：9/9 groups PASS；完整最差指标见 [`2026-08-25-validation.md`](evidence/automated/2026-08-25-validation.md)。
- Coordinate contract：PASS。
- ROS/Jazzy：4 packages build PASS；6/6 Adapter gtest PASS；test result `18 tests, 0 errors, 0 failures, 0 skipped`。
- Replay：250 samples，finite、bounded、逐样本 deterministic；最大绝对重复差为 `0`。

## Decisions Confirmed

- 正式单腿 plant 为固定基座、无接触的五刚体闭链，含 hip/knee/wheel 三路 actuator、两路被动关节和 3 维独立运动子空间。
- 完整 plant 的 unconstrained mass matrix 为 16×16；base weld 与双 closure 的 constraint Jacobian rank 为 10，nullspace 为 6 维。
- MuJoCo 原生 actuator `+ctrl` 产生原生关节正加速度；canonical Controller 力矩仍由 Adapter 按 `tau_M=-tau_C` 转换。
- 重力、完整闭链约束、纯惯性耦合和能量/replay 使用隔离工况分别验证，不用一个混合轨迹替代各自证据。
- PASS 仅表示 nominal MuJoCo 内部自洽；所有真机准确性结论继续需要共同辨识。

## Deviations from PLAN

- 执行中发现并修复 Phase 04 模型的截断欧拉角问题；按照 PLAN 规定记录 finding、重新计算 offset 并完成 Phase 04 回归。
- 单腿 fixture 最终保留全部五个刚体、闭链和被动 DoF；开链分析通过显式关闭 closure 隔离，未使用不可追溯临时改模。
- Energy/replay 使用零重力隔离纯惯性功—能收支；重力与完整闭链使用独立正式 gate 验证。

## Known Limitations and Follow-ups

- Imported mass/COM/inertia 未经真机验证；damping/frictionloss/armature 为零默认值，真实值 unknown。
- Unit-gear actuator 不是电机/驱动器标定模型；wheel-floor contact fidelity 未验证。
- 下一步可恢复 [Phase 05 执行器力矩辨识与模型校准](../05-actuator-torque-identification/PLAN.md)，但仍须先关闭其通信、Load Cell、同步和安全包络 gate，不能直接上正式实验。

## ROADMAP Update

- 本 Phase 对应阶段：ROADMAP 顺序 05“MuJoCo 运动学与内部动力学验证”。
- 状态变化：`planned → active → review → complete`。
- 下一建议 Phase：[Phase 05 执行器力矩辨识与模型校准](../05-actuator-torque-identification/PLAN.md)。

## Key Links

- [PLAN](PLAN.md)
- [REVIEW](REVIEW.md)
- [ROADMAP](../../ROADMAP.md)
