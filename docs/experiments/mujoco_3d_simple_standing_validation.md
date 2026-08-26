# MuJoCo nominal 完整 3D 简单站立验证

## 目的与边界

本方法验证 current nominal `phase18_floating_contact.xml` 上的 simulation-only 简单站立。控制器只使用 canonical `RobotState`，输出 canonical 六关节 `TorqueCommand`；不使用运行时 MuJoCo 动力学、隐藏约束、侧向外力控制、WBC/QP/NMPC 或真机数据。

## 冻结对象

- plant：完整 freejoint、左右闭链、六 actuator、wheel-only contact；floating reset 后 `base_weld` inactive。
- timing：physics `0.002 s`，control `0.010 s`，每个 command 严格保持五个 physics step。
- equilibrium：[`phase20_equilibrium.json`](../../simulation/mujoco/config/phase20_equilibrium.json) 的 zero-wheel-torque upright 解。
- state：`[x-x0,vx,pitch,omega_y,roll,omega_x,yaw-yaw0,omega_z]`；姿态使用 world-axis shortest-arc quaternion Log。
- virtual input：`[common wheel,roll leg,yaw wheel]`；数值 gain、roll direction、case matrix与门槛全部来自 [`phase20_formal.json`](../../simulation/mujoco/config/phase20_formal.json)。
- controller：固定腿参考的 support+PD 与静态 `u=-Kx`；任何安全门或六路 torque limit 触发均输出全零并锁存至 reset。

## 前置门槛

Core/formal 只在下列独立证据通过后执行：

1. equilibrium solver 与 fresh replay：qacc、generalized residual、closure、双轮normal load、one-step drift和hash通过。
2. state/input contract：完整3D编译维度、freejoint、orientation Log、三路input rank/condition/sign和roll cross-coupling通过。
3. nonlinear pre-freeze：10 ms中心差分模型可控秩为8；独立轨迹fit门通过；所有tuning与未参与选择的正负/组合holdout在10 s内通过。

探索或失败目录不得覆盖。只有 pre-freeze summary 的decision为`IMPLEMENT_CORE`才准入正式链。

## 正式执行

先构建并测试 C++ 边界：

```bash
cd ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to wheel_leg_mujoco
colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco
colcon test-result --verbose
```

再把冻结case matrix写入一个全新的 evidence 目录：

```bash
./.venv/bin/python tools/experiments/run_mujoco_3d_standing_formal.py \
  --output-dir docs/workflow/phases/20-nominal-3d-simple-standing/evidence/automated/<run-id>
```

wrapper 必须在启动首个仿真前拒绝非空目录。每个正常case运行至少1000个control ticks；fault case运行双episode并在reset后exact replay。fresh replay使用同一冻结输入写入另一个新目录，不能复用或覆盖首次输出。

## 判定

正常case逐tick检查：finite、controller/Adapter接受状态、`X/Y/Z`、roll/pitch/heading、世界线/角速度、腿姿态与速度、bilateral wheel contact、六路raw/command/native torque、virtual-input分解、Adapter sign、5-step ZOH及最终恢复。

fault matrix分别注入left/right contact loss、invalid quaternion、nonmonotonic time、错误control period和torque saturation。注入tick必须得到预期status；从该tick至episode结束六路command必须严格为零且latch保持；下一episode reset必须恢复并与首episode故障前轨迹一致。

只有所有case与fault check、manifest/hash、fresh replay、历史回归全部通过，REVIEW才可判定`PASS`。动画和plot仅用于观察，不参与PASS。
