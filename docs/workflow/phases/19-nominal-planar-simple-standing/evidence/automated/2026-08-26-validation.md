# Phase 19 自动验证记录（2026-08-26）

## Result

`PASS`。全部结果仅属于 current nominal exact-planar MuJoCo simulation；未连接或操作真机。

## Pre-freeze

```bash
./.venv/bin/python tools/experiments/run_mujoco_planar_prefreeze_v3.py
./.venv/bin/python tools/experiments/run_mujoco_planar_prefreeze_v3.py \
  --output-dir docs/workflow/phases/19-nominal-planar-simple-standing/evidence/exploratory/2026-08-26-planar-prefreeze-v3-replay
```

- `Kp/Kd=12/1.5` 在 contact plant 的 10 ms sampled leg loop 中复现失败；standing profile 改为 `8/1`，不改变 2 ms / 10 ms / 5-step ZOH。
- local model rank `4`，闭环谱半径 `0.9847891283`，`0.5×/1×/2×` A/B 最大差 `7.05e-8`。
- raw full-coordinate Jacobian 离开 equality/contact constraint manifold，谱半径随步长在 `1.1102–10.0081` 变化，只保留为诊断，不作为 release oracle。
- primary/replay summary SHA-256 均为 `7148e46aa2547342d8c02c81488a2568ba310c45264df94124ff96fe06e97214`。

## C++ formal-v4

```bash
cd ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to wheel_leg_mujoco
cd ..
./.venv/bin/python tools/experiments/run_mujoco_planar_standing_formal.py
./.venv/bin/python tools/experiments/run_mujoco_planar_standing_formal.py \
  --output-dir docs/workflow/phases/19-nominal-planar-simple-standing/evidence/automated/2026-08-26-formal-v4-replay
```

- 11 个 normal/perturbation cases 全部运行 10 s：nominal/reset replay、`±0.005 rad` contact-projected pitch、`±0.01 m/s` rolling、`±0.2 N × 0.1 s` X force、`±0.02 N·m × 0.1 s` pitch moment、四主动关节 `±0.002 rad`。
- 4 个 fault cases 各运行 2 episodes：contact loss、invalid state、nonmonotonic timestamp、torque saturation；全部在 fault tick 零输出并锁存，reset 后 exact replay。
- 全矩阵最大 pitch `0.005 rad`、X error `0.004176 m`、height error `0.000409 m`、active-leg error `0.012130 rad`。
- 所有采样点双轮接触；左右轮 canonical torque 差、Adapter sign error、五步 ZOH ctrl 差均精确为 `0`。
- primary/replay summary SHA-256 均为 `adb1fd39d1ce21e47da49cf1352170d2e03190f103ae865d42a6a0ab25a4f373`；formal-v4 manifest 额外冻结 Core header/source、Adapter、C++ runner、CMake、binary、config、wrapper 与 scene hashes。
- 再次指向正式非空目录返回 exit `2`，在仿真前拒绝覆盖。

`formal/` 是首轮带 startup contact grace 的已废弃结果；`formal-v2/` 是收紧首帧接触后暴露非法 `+0.01 rad` reset 的失败结果；`formal-v3/` 与 v4 数值相同但 manifest 尚未列出全部源码 hashes。它们均保留，正式 authority 仅为 `formal-v4/`。

## Build/tests and regressions

- `colcon build --symlink-install --packages-up-to wheel_leg_mujoco`：4 packages PASS。
- `colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco`：19 tests，0 errors/failures/skipped。
- Phase 02 coordinate contract PASS。
- Phase 14 internal dynamics 9/9 groups PASS。
- Phase 15 closed-chain kinematics 8/8 groups PASS。
- Phase 16 deterministic loop 24/24 checks PASS。
- Phase 17 Joint PD+gravity overall PASS。
- Phase 18 contact/floating-base 20/20 checks PASS。
- fresh revision-workflow namespace 重新执行 source→planar generator、equilibrium、v3 pre-freeze，全 PASS；preserved compiled field、initial pose 与 option 最大差均为 `0`。

## Limits

- Formal pitch 初态为保持双轮首帧接触的 `±0.005 rad`；旧 `±0.01 rad` 固定腿 reset 不在接触流形上，不作为正式初态。
- 外力/力矩是冻结的小扰动包络，不代表大扰动恢复或 region of attraction。
- 不支持完整 3D、roll/yaw、转向、单轮支撑、WBC、NMPC、真实接触或真机站立结论。
