# MuJoCo 内部动力学验证方法

## 目的与边界

本方法为 Phase 14 提供可重复的 MuJoCo 3.7.0 内部自洽验证。它只证明当前 nominal plant 的运动学、质量矩阵、动力学、约束、能量和确定性计算彼此一致，不证明模型与真机一致，也不使用任何真机设备或数据。

## 固定输入

- 完整无接触 plant：`simulation/mujoco/model/phase14_contact_free.xml`；保留自由基座、`base_weld` 和左右闭链，删除地面并禁用 contact。
- 单腿 plant：`simulation/mujoco/model/phase14_single_leg.xml`；固定基座、无接触的五刚体闭链，保留 hip/knee/wheel 三路 actuator 与两路被动关节，惯性取自完整模型的 MuJoCo 3.7.0 编译结果。开链子测试显式关闭 closure，只分析 hip→knee→wheel 支路。
- 版本化采样、seed、激励和阈值：`simulation/mujoco/config/phase14_validation.json`。
- Canonical 映射保持 `q_C=-q_M+b`、`dq_C=-dq_M`、`tau_M=-tau_C`；结果中的 `ctrl_native` 是 MuJoCo 原生 actuator 力矩，不是 canonical `TorqueCommand`。

## 验证项

1. Fixture invariant：按名字核对 DoF、actuator、equality、contact-free 和 fixed-base 单腿边界。
2. FK/Jacobian：用独立齐次变换和解析叉乘 Jacobian 对比 MuJoCo body pose/Jacobian，覆盖零位、典型姿态、边界姿态和两个冻结随机样本。
3. Gravity：用编译 body mass/COM 的势能有限差分对比 `qfrc_bias`，并验证自由落体方向和完整广义力静态平衡。
4. Mass：检查完整 `M(q)` 和 driven submatrix 的对称性、特征值、条件数；用 equality Jacobian nullspace 明确 constrained 口径。
5. Forward/inverse：检查 `qfrc_inverse = M qdd + bias - passive`，并把 inverse 输出送回 forward dynamics。
6. Constraint：完整双腿、base weld 和两条 closure 全开，关闭 gravity/contact/input，运行 100 步检查残差和 finite state。
7. Coupling：单腿 plant 逐 actuator 注入 `+1 Nm`，检查 generalized-force mapping、正对角响应、非零耦合和 reciprocity。
8. Energy/replay：零重力、无接触、固定基座条件下施加冻结的 0.002 Nm 正弦输入；检查功—能收支、bounded state 和逐样本确定性。

重力、闭链和能量分别验证，避免把无阻尼自由摆落、约束稳定化损失和 actuator 功混入同一个指标。

## 执行

```bash
cd /home/t/W_L_ws
./.venv/bin/python tools/experiments/run_mujoco_internal_dynamics.py \
  --output-dir data/experiments/2026-08-25-mujoco-internal-dynamics/raw
./.venv/bin/python tools/experiments/run_mujoco_internal_dynamics.py
```

脚本任一 gate 失败即返回非零。正式阈值随配置文件版本化；不能根据正式结果静默放宽。

## 输出与通过条件

- `phase14_validation.json`：每项 gate、最差值、阈值输入 hash 和总结果。
- `parameter_manifest.json`：编译模型逐 body/joint/actuator 参数及 provenance/status。
- `open_loop_replay.csv`：`time/q/dq/qdd/ctrl_native/constraint residual/energy`，SI 单位。

只有 JSON 中 `overall_pass=true`、Phase 04 回归继续通过、证据审查没有 blocking finding 时，Phase 14 才能 PASS。
