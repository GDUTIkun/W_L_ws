# Phase 16 自动验证记录（2026-08-25）

## 正式 deterministic run

命令：

```bash
cd /home/t/W_L_ws
source /opt/ros/jazzy/setup.bash
./.venv/bin/python tools/experiments/run_mujoco_controller_loop.py \
  --profile nominal \
  --output-dir docs/workflow/phases/16-controller-mujoco-deterministic-loop/evidence/automated/2026-08-25-nominal-v2
```

结果：`PASS`，`overall_pass=true`，24/24 gate 通过。最终审查解除 `--profile` 的 nominal-only CLI 限制后，重新运行的正式输出位于同目录的 `2026-08-25-nominal-v2/`；原 `2026-08-25-nominal/` 保留且未覆盖：

- 2 episode × 100 control ticks，全部行有限；2 ms physics、10 ms control、每 tick 5 physics steps 无漂移。
- Core 最大绝对 torque `0 N·m`；native ctrl 最大绝对值 `0 N·m`；ZOH 最大差 `0 N·m`。
- reset/fresh 最大数值差 `0`；fresh/fresh 最大数值差 `0`，两个 CSV SHA-256 均为 `f0b310075659d06ab2088fbdd4567de9c7404e14392b8aec7eea3e52e7432e9a`。
- duplicate/future/stale/receipt-timeout/reset-old 均按预期拒绝或归零；timeout 后和新 epoch 的合法命令恢复。
- `hardware_data_used=false`。本结果不支持 PD、接触、实时性或真机结论。

正式 manifest 的 8 个输入 hash 与 3 个输出 hash 在最终构建后重新计算，全部匹配。对同一正式目录重复运行返回 exit 1，并报告 `Refusing to overwrite non-empty output directory`。

## Build and package regressions

```bash
cd /home/t/W_L_ws/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to wheel_leg_mujoco
colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco \
  --event-handlers console_cohesion+
colcon test-result --verbose
```

结果：4 packages build PASS；18 tests、0 errors、0 failures、0 skipped。`test_pubsub` 固定到 `ROS_DOMAIN_ID=232`，避免接收用户正在运行的默认 Domain 仿真消息；用户进程没有被停止或修改。

## ROS compatibility smoke

在独立 `ROS_DOMAIN_ID=231` 启动 `zero_loop.launch.py floating_base:=false`，随后读取一次 `/robot_state` 和 `/torque_command`，并依次调用 `/reset_simulation`、`/reset_controller`。

结果：RobotState 字段有限；TorqueCommand 六路均为 `0 N·m`；两个 reset service 均 `success=true`；launch 收到 SIGINT 后两个节点 clean exit。该检查只证明 transport/schema/reset compatibility。

## Historical regressions

```bash
cd /home/t/W_L_ws
./.venv/bin/python tools/maintenance/test_mujoco_coordinate_contract.py
./.venv/bin/python tools/experiments/run_mujoco_internal_dynamics.py \
  --output-dir data/experiments/2026-08-25-phase16-phase14-regression/raw
./.venv/bin/python tools/experiments/run_mujoco_closed_chain_kinematics.py \
  --output-dir data/experiments/2026-08-25-phase16-phase15-regression/raw
```

结果：coordinate contract PASS；Phase 14 的 fixture/kinematics/gravity/mass matrix/forward-inverse/constraints/coupling/energy/replay 全部 PASS；Phase 15 的 geometry/assembly/workspace/FK/full/reduced Jacobian/velocity/virtual work/symmetry/determinism 全部 PASS。

## Observed regression issue and resolution

首次 ROS pub/sub regression 使用默认 Domain，测试订阅到了用户另一个终端中持续运行的 `/wheel_leg_controller`，导致接收计数高于测试预期。这不是 Controller 功能失败。测试改为先发现端点再发送单样本，并由 CTest 固定独立 Domain；相同外部进程仍运行时回归通过。
