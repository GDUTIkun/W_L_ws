# Controller ↔ MuJoCo 确定性闭环验证方法

## 目的与边界

本方法验证 current nominal MuJoCo plant、canonical `RobotState`/`TorqueCommand`、Controller Core 和 Adapter 能以固定步序运行、复位、重放和留证。当前 Core 对有效状态只输出六路零力矩，因此 PASS 只说明执行链、时间、命令生命周期、日志和确定性正确，不说明 PD、站立、接触保真度、实时性或真机一致性。

## 固定输入

- Scene：`simulation/mujoco/model/phase16_contact_free.xml`；包含 current nominal `wheel_leg.xml`、双腿闭链和 `base_weld`，保留名为 `floor` 的 Adapter 对象，同时全局关闭 contact。
- Config：`simulation/mujoco/config/phase16_nominal.json`。
- Physics period：`0.002 s`；Control period：`0.010 s`；每个控制命令零阶保持 5 个 physics steps。
- Runtime：`wheel_leg_mujoco/deterministic_loop` 直接链接现有 Controller Core 与 Adapter；不经过 ROS callback，也不复制 joint mapping。
- Python wrapper 只启动 C++ runner、解析 CSV、检查 gate、计算 SHA-256 和写 JSON，不实现第二套物理或控制循环。

## 唯一步序与时间语义

每个 control tick `t_k` 严格执行：

```text
mjData(t_k)
  → Adapter.extractState
  → ControllerCore.step
  → Adapter.acceptCommand
  → [Adapter.writeControls → mj_step] × 5
  → 下一 control tick
```

`RobotState.sample_time_ns` 和 `TorqueCommand.source_sample_time_ns` 来自 MuJoCo 仿真时间；Adapter receipt time 是单独的单调逻辑时钟。日志分别保存两者，禁止跨时钟域相减。episode 开始时先 reset MuJoCo/Adapter，再 reset Controller；旧 episode 命令不得进入新 epoch。

## Nominal 与故障场景

Nominal 正式运行包含两个 100-control-tick episode，并在两个新 C++ 进程中各执行一次。检查逐 tick 数量、source/receipt time、Controller `dt`、5-step accounting、有限值、零 Core torque、零 native ctrl、ZOH、同进程 reset replay 和跨进程 CSV 等价。

Fault 场景包含 duplicate state、future/stale command、receipt timeout、timeout 后恢复以及 reset-old command。为使 Adapter timeout 路径可观察，runner 在 `timeout_seed` 处向 Adapter 注入一条明确标记的 `1 N·m` 实验命令；该命令不来自 Core，Core 输出仍记录为零。该路径只验证 Phase 04 已冻结的映射/watchdog，不构成控制算法。

## 执行与输出

先构建 executable：

```bash
cd /home/t/W_L_ws/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to wheel_leg_mujoco
```

再使用新的输出目录执行：

```bash
cd /home/t/W_L_ws
./.venv/bin/python tools/experiments/run_mujoco_controller_loop.py \
  --profile nominal \
  --output-dir data/experiments/<new-phase16-run-id>/raw
```

- `nominal_a.csv`、`nominal_b.csv`：两个 fresh process 的完整 control-tick 日志。
- `faults.csv`：故障注入、拒绝、归零和恢复日志。
- `phase16_validation.json`：逐 gate 结果、最差指标与解释边界。
- `run_manifest.json`：模型 revision、timing、环境版本、输入/runner/output SHA-256。

非空输出目录会被拒绝。模型、Controller 或 config 改变后必须使用新 run ID；不能覆盖本次 nominal evidence，也不能继承其 bitwise PASS。
