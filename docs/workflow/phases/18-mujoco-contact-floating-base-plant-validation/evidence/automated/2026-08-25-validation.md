# Phase 18 Automated Validation — 2026-08-25

## Formal contact/floating matrix

命令：

```bash
./.venv/bin/python tools/experiments/run_mujoco_contact_floating_base.py \
  --output-dir docs/workflow/phases/18-mujoco-contact-floating-base-plant-validation/evidence/automated/2026-08-25-formal-v5
```

结果：PASS，20/20 checks。关键最差值：

- probe static load relative error：`9.846899650131914e-05`；阈值 `0.01`。
- probe vertical impulse error：`2.150280057691134e-04 N·s`；阈值 `0.005 N·s`。
- probe max penetration：`0.003296063277396004 m`；阈值 `0.004 m`。
- positive rolling displacement：left `0.14814935897320633 m`、right `0.14268518988738588 m`；negative 均为负；frictionless 绝对值小于 `5e-05 m`。
- lateral final speed：`mu=0` 为 `0.2999058285730666 m/s`，`mu=1/2` 收敛到数值零。
- max positive friction power：`0.000668495993320626 W`；容差 `0.001 W`。
- full-model first contact：step `26`（`0.052 s`）；free-fall acceleration error `8.129800569740553e-04 m/s²`。
- full-model max penetration `0.0032409424655467714 m`、closure residual `0.0001153163729117319 m`、quaternion norm error `7.77e-16`。
- unexpected contact pair：0；同进程 reset replay exact。

正式 raw/summary/manifest：[`2026-08-25-formal-v5`](2026-08-25-formal-v5/phase18_validation.json)。

## Fresh-process and non-overwrite

- 第二进程新目录生成的 11 个 CSV 与 formal-v5 SHA-256 全部一致。
- 再次指向 formal 默认非空目录：exit `1`，在仿真前报告 `Refusing to overwrite non-empty output directory`。

## Build/tests and regressions

- `colcon build --symlink-install --packages-up-to wheel_leg_mujoco`：PASS。
- `colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco`：19 tests，0 errors/failures/skipped。
- Phase 02 coordinate contract：PASS。
- Phase 14 internal dynamics：9/9 groups PASS。
- Phase 15 closed-chain kinematics：8/8 groups PASS。
- Phase 16 deterministic loop：24/24 checks PASS。
- Phase 17 Joint PD+gravity：14/14 checks PASS。

## Interpretation limit

以上只证明 current nominal MuJoCo wheel-contact 与零控制 floating-base touchdown 在冻结工况内一致、可重复。没有真实轮胎/地面数据，不形成站立、真实 friction/compliance、滚阻、执行器或真机结论。
