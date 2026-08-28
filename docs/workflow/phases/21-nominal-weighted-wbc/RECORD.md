# Phase 21: nominal Weighted WBC — RECORD

Status: `complete`

## Outcome

current nominal完整3D MuJoCo plant已在canonical C++ Controller Core上完成simulation-only Weighted WBC。runtime-independent 12-DoF reduced model、42变量/104 hard-row QP、连续contact-centred wrench、weighted standing tasks与interaction-wrench slack均已冻结；正式10 s case matrix、fault/reset、plant/contact、fresh replay、non-overwrite、历史回归与最终REVIEW全部PASS。

## Delivered

- canonical state到闭链passive reconstruction、reduced dynamics/contact/wrench map的C++ nominal model。
- fixed-order 42D weighted problem与project-owned Eigen-only dense ADMM solver wrapper。
- additive `ControllerCore::kWeightedWbc` mode、nominal reference producer、solver/model/task diagnostics和fail-zero latch/reset。
- 独立full-3D `weighted_wbc_loop`，保持Core↔Adapter、2 ms physics/10 ms control/5-step ZOH及plant-truth隔离。
- versioned model/hard/task/formal profiles、formal evaluator、primary/replay manifest/hash及历史reuse证据。

## Key Results

- solver：cold/dynamic最大总时长`8.273542/8.790942 ms`，最大hard/equality/stationarity`1.128e-7/1.128e-7/4.124e-8`，最大物理力矩差`3.075e-5 N·m`。
- pre-freeze：4个workspace、28个dynamic hard problems、4个10 s tuning和9个holdout全部PASS，workspace failure/violation为0。
- formal-v1：19/19 normal/perturbation、6/6 fault PASS；bilateral contact fraction`1.0`，零饱和。
- worst state：X/Y`2.075e-3/2.006e-3 m`、height`1.680e-4 m`、roll/pitch/yaw`6.377e-3/7.181e-3/1.977e-2 rad`、leg`1.480e-2 rad`。
- worst runtime/plant：Core step`9.90157 ms`、minimum load`31.27 N`、penetration`5.369e-4 m`、rolling/lateral slip`8.272e-3/1.731e-3 m/s`、closure`1.835e-4 m`。
- QP/task：hard/primal/dual/stationarity`1.070e-7/1.265e-7/6.506e-8/4.205e-8`，slack`3.728e-3`，task residual/cost`5.523e-3/4.290e-5`。
- primary/replay 25个plant CSV字节一致，control仅墙钟列不同；两套manifest合计134项hash复核无漂移；ROS workspace `24 tests, 0 failures`。

## Decisions

- plant authority保持current nominal `phase18_floating_contact.xml`；production Core/WBC不得链接或读取MuJoCo private state/plant truth。
- reduced velocity固定为world-axis base twist加六active velocity；passive state必须在Phase15冻结branch/workspace内重构，否则fail closed。
- decision vector固定为`[nudot_12,tau_6,w_left_C6,w_right_C6,slack_left_FLU6,slack_right_FLU6]`，hard rows固定104；interaction wrench满足`W_feasible=W_reference+slack`。
- solver固定`alpha=1.6`、weighted wrapper `rho=0.15`；任何错误、超时、非有限、hard violation或limit失败均六路zero并锁存到reset。
- 保持`0.002 s / 0.010 s / 5-step ZOH`和opt-in WBC mode；不改变公共`RobotState/TorqueCommand`或旧Controller modes。
- formal-v1及其fresh replay是最终authority；历史36D single-force合同只作回归，不再具有current runtime authority。

## Evidence

- [Formal summary](evidence/automated/2026-08-28-formal-v1/summary.json)
- [Formal manifest](evidence/automated/2026-08-28-formal-v1/manifest.json)
- [Fresh replay summary](evidence/automated/2026-08-28-formal-v1-replay/summary.json)
- [Formal audit](evidence/automated/2026-08-28-formal-v1/README.md)
- [Runtime C++ parity](evidence/runtime_cpp_parity.md)
- [Workspace gate repair](evidence/runtime_workspace_gate_repair.md)
- [Validation method](../../../experiments/mujoco_weighted_wbc_validation.md)
- [Review](REVIEW.md)

## Reproduction

```bash
cd /home/t/W_L_ws/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to wheel_leg_mujoco
colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco
colcon test-result --verbose
cd ..
./.venv/bin/python tools/experiments/run_mujoco_weighted_wbc_formal.py \
  --output-dir docs/workflow/phases/21-nominal-weighted-wbc/evidence/automated/<new-run-id>
```

输出目录必须为空。任何model/profile/config变更都必须写入新run namespace并重走model、hard QP、task和formal gates，不得覆盖本RECORD引用的authority。

## Limits and Next Use

- 不证明NMPC、真机、identified/new CAD profile、target hardware实时性、terrain、单轮支撑、大扰动或跌倒恢复。
- `9.90157 ms`只满足当前simulation host冻结门槛，余量薄；部署前必须在目标硬件独立测量deadline与调度抖动。
- Phase 22可以替换内部standing reference producer接入NMPC wrench command，但不得改变本Phase冻结的WBC sign/order/slack及canonical public I/O；必须建立独立PLAN和证据。
