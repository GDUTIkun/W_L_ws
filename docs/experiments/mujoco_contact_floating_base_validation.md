# MuJoCo 轮地接触与 floating-base plant 验证方法

## 目的与边界

本方法验证 current nominal MuJoCo 的 wheel-only contact 和零控制 floating-base plant。它不连接真机、不控制腿、不验证站立，也不把 nominal friction/compliance 解释为真实轮胎参数。

## 两层工况

1. `phase18_wheel_contact_probe.xml` 使用左右真实 wheel collision mesh、三向平移 carriage 和 wheel hinge，隔离验证法向支撑、正负滚动、横向滑移及 `mu=0/1/2` 趋势。
2. `phase18_floating_contact.xml` 使用完整闭链机器人，关闭 `base_weld`、六路 ctrl 保持零，从指定 base height 自由落体并在 `0.2 s` 窗口内验证触地、base quaternion、整机 COM、闭链和 reset replay。

所有 physics row 使用 `0.002 s` 步长。probe 的正 wheel torque 遵循 Phase 15 契约，对应轮心 `+X` 前滚；lateral 为 world `Y`，normal 为 world `Z`。

## Contact wrench

`mj_contactForce` 的 contact-frame force 经 `contact.frame^T` 转到 world FLU，再依据 wheel 位于 `geom1` 或 `geom2` 统一为“floor 作用在 wheel 上”的力。该符号由以下独立关系验证：

- 静止后平均 `Fz ≈ subtree_mass * g`；
- `∫Fz dt ≈ m(vz_final-vz_initial+gT)`；
- friction cone 内切向力有界，切向力与接触点 slip 的功率不超过数值容差。

## 正式入口

```bash
cd /home/t/W_L_ws
./.venv/bin/python tools/experiments/run_mujoco_contact_floating_base.py \
  --config simulation/mujoco/config/phase18_nominal.json \
  --output-dir data/experiments/<new-phase18-run-id>/raw
```

runner 拒绝非空输出目录。每次运行保存 11 个逐物理步 CSV、`phase18_validation.json` 和带输入/输出 SHA-256 的 `run_manifest.json`。

## 解释限制

- `condim=3` 只验证法向与二维 sliding friction，不验证真实滚阻或扭转摩擦。
- 完整机器人只看首次触地后的 bounded window；无腿控制的长期倒塌不是 Phase 18 的失败，也不能被描述为站立。
- identified profile 必须用新 config/new run 追加，不能覆盖 nominal evidence。
