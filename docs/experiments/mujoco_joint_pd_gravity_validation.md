# MuJoCo Joint PD 与名义重力补偿验证方法

## 边界

对象是 current nominal、fixed-base、contact-disabled 双闭链 plant。控制周期 `10 ms`，物理周期 `2 ms`，命令 5-step ZOH。本方法不验证轮地接触、floating-base、站立、真实执行器或真机安全值。

## 控制律

显式 profile 使用 canonical measured state：

```text
tau_pd  = Kp (q_ref - q) + Kd (dq_ref - dq)
tau_raw = tau_pd + tau_g
tau_cmd = clamp(tau_raw, -limit, +limit)
```

current nominal gains 每侧为 hip/knee/wheel `Kp=[12,12,0.3]`、`Kd=[1.5,1.5,0.05]`；limits `=[6,6,1] N·m` 仅用于本次 ideal-actuator 仿真。

## 重力 profile 与 oracle

平面刚体势能在 nominal closure branch 上约化为每腿三项：native `q_h`、`q_h+q_k`、`q_h+q_k+q_w` 的正弦/余弦 generalized-torque 系数。第三项来自真实 compiled wheel COM 小偏心，不能清零。Core 由 `q_M=b-q_C` 计算解析项，不链接 MuJoCo。

离线验证在 Phase 15 冻结工作域取 150 个姿态，逐点比较：

1. full `qfrc_bias` 经闭链 reduction `S` 投影并转换到 canonical sign；
2. 沿闭链 branch 对 compiled rigid-body potential 做中心差分；
3. C++ tick 日志的 `tau_gravity` 与 versioned JSON profile 逐点一致。

## 正式矩阵

- zero、gravity-only 短时、PD-only hold、PD+gravity hold；
- 六个关节的正/负阶跃，hip/knee `0.1 rad`、wheel `0.25 rad`；
- 三类左右同向对称阶跃；
- 三类 10 ms 外部 joint-torque pulse 后恢复；
- 求和后饱和、episode reset replay、fresh-process replay；
- 所有日志有限、5-step interval 内 ctrl 不变。

执行：

```bash
cd /home/t/W_L_ws/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to wheel_leg_mujoco

cd /home/t/W_L_ws
./.venv/bin/python tools/experiments/run_mujoco_joint_pd_gravity.py \
  --output-dir data/experiments/<new-phase17-run-id>/raw
```

输出目录必须为空。`phase17_validation.json` 给出 gate/指标，`run_manifest.json` 保存版本和输入/输出 SHA-256，CSV 保留逐 tick reference、PD、gravity、raw、command、saturation 与 disturbance。
