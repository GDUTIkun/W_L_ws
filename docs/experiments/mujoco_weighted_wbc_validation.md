# MuJoCo nominal 完整 3D Weighted WBC 验证

## 目的与边界

本方法验证 current nominal `phase18_floating_contact.xml` 上的 simulation-only
12-DoF reduced-model 42 变量 Weighted WM-WBC。Controller 只使用 canonical
`RobotState`（含左右 contact enum 与六关节状态），输出 canonical 六关节
`TorqueCommand`；production Core/WBC 链路不链接、不读取 MuJoCo 动力学。不使用
NMPC、隐藏外力、隐藏约束、真机数据或 Simulink 数值参数。

## 冻结对象

- plant：完整 freejoint、双闭链、六 actuator、wheel-only contact；floating
  reset 后 `base_weld` inactive（runner 的 `adapter.reset` 负责，`setInitialState`
  不得再次 `mj_resetData`）。
- timing：physics `0.002 s`，WBC control `0.010 s`，每个 command 严格保持五个
  physics step（5-step ZOH，逐 tick `zoh_diff == 0`）。
- equilibrium/reference：[`phase20_equilibrium.json`](../../simulation/mujoco/config/phase20_equilibrium.json)
  的 zero-wheel-torque upright 解；canonical leg reference 与 nominal wrench
  profile 来自 P21-T06 冻结值（已编入 `currentNominalWeightedWbcConfig()`）。
- decision vector：`z=[nudot_12,tau_6,wL_C_6,wR_C_6,slackL_FLU_6,slackR_FLU_6]`，
  42 变量/104 hard rows；solver 为 project-owned Eigen-only dense ADMM
  （`alpha=1.6`，weighted wrapper `rho=0.15`），episode 内 warm、reset 后 cold。
- formal 输入：[`phase21_weighted_wbc_formal_v1.json`](../../simulation/mujoco/config/phase21_weighted_wbc_formal_v1.json)
  （case matrix、fault matrix、全部 gates、solver 字段与源 profile 清单），运行前
  冻结；失败后必须新建 profile/run 并记录 supersedes，不得原地改参覆盖。
- case matrix：Phase 20 的 19 个 normal/perturbation case（nominal 2-episode
  reset replay、pitch/velocity/yaw-rate 初态扰动、force_x/force_y、三类 moment、
  combined 正负）与 6 个 fault case（left/right contact loss、invalid
  quaternion、nonmonotonic time、timing、torque saturation）。扰动量级、注入
  tick、normal/fault tick 数与 Phase 20 formal 完全一致。

## 前置门槛

formal 只在下列独立证据通过后执行：

1. P21-T05 hard layers/equilibrium（DG21-04）与 P21-T06 weighted tasks、slack
   与 10 s nonlinear tuning/holdout（DG21-05）全部 PASS。
2. P21-T07 runtime model/QP/solver 与 offline oracle 逐项一致，deadline 审计
   通过（DG21-03）。
3. P21-T08 Core `kWeightedWbc` mode 测试与 P21-T09 runner 验收（Adapter、
   5-step ZOH、双时钟、fault/reset、replay、non-overwrite）通过。

## 正式执行

先构建并测试 C++ 边界：

```bash
cd ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to wheel_leg_mujoco
colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco
colcon test-result --verbose
```

再把冻结 case matrix 写入一个全新的 evidence 目录：

```bash
./.venv/bin/python tools/experiments/run_mujoco_weighted_wbc_formal.py \
  --output-dir docs/workflow/phases/21-nominal-weighted-wbc/evidence/automated/<run-id>
```

wrapper 必须在启动首个仿真前拒绝非空目录；runner 自身拒绝已存在的输出路径
（non-overwrite 双保险）。每个正常 case 运行 1000 个 control ticks（10 s）；
fault case 运行 200 ticks × 双 episode 并在 reset 后 exact replay。fresh replay
使用同一冻结输入写入另一个新目录，不能复用或覆盖首次输出。

## 判定

正常 case 逐 control tick 检查：finite、Core `kOk` 且零 latch、WBC/model/solver
status 全 `kOk`、command 被 Adapter 接受、`dt==0.010 s`（tick 0 为 0）、
hard/primal/dual/stationarity residual ≤ `2e-7`、normalized slack ≤ `0.01`、
task residual/cost ≤ `0.02/0.001`、单拍 Core 步时 ≤ 10 ms（deadline）、六路
command 无饱和、5-step ZOH 与 Adapter 符号（`ctrl==-tau`）精确为零。

逐 physics substep 检查 plant truth：双轮 normal load ≥ 1 N、penetration
≤ 4 mm、rolling/lateral slip ≤ 0.05 m/s、closure ≤ 2e-4 m、双轮接触率 1.0。
状态 envelope：X/Y ≤ 0.02 m、高度误差 ≤ 0.01 m、roll/pitch ≤ 0.03 rad、
yaw ≤ 0.05 rad（world-axis shortest-arc Log，相对每 episode 首拍锚）、leg
误差 ≤ 0.03 rad、末拍线速度 ≤ 0.02 m/s、角速度 ≤ 0.1 rad/s。多 episode
case 另要求 control CSV（排除 `core_step_ns` 墙钟列）与 plant CSV 的 episode
间 exact replay。

fault matrix 分别注入 left/right contact loss、invalid quaternion、
nonmonotonic time、错误 control period 与 torque saturation（以 0.001 N·m
六路 limit 从首拍注入）。注入 tick 必须得到预期 status（invalid→`kInvalidState`、
nonmonotonic→`kNonMonotonicState`、其余→`kSafetyLatched`）；从该 tick 至
episode 结束六路 command 必须严格为零且 latch 保持；下一 episode reset 必须恢复
并与首 episode exact 一致。

manifest 记录 config/runner/wrapper/scene、Core 与 runner 源码、全部依赖
profile 的 SHA-256，以及全部输出 CSV/summary 的 hash；`solver` 块记录变量数、
hard 行数、decision order、`alpha/rho` 与 warm/cold 语义。缺任一字段、空
case matrix 或非空输出目录都必须在写入前失败。

只有所有 case 与 fault check、manifest/hash、fresh replay、Phase 14/15/18/20
历史回归全部通过，REVIEW 才可判定 `PASS`。动画和 plot 仅用于观察，不参与 PASS。
