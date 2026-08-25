# Phase 18: nominal 轮地接触与 floating-base plant 验证 — RECORD

Status: `complete`

## Outcome

current nominal MuJoCo 已具备显式 wheel-only collision profile，并通过 actual-wheel contact probe 与完整机器人零控制 floating-base touchdown 验证。结论限定为 MuJoCo 内部一致性，不代表站立或真实接触参数。

## Delivered

- `wheel_leg.xml`：普通 imported CAD mesh 禁止碰撞；左右命名 wheel geom 显式使用 `(contype=0, conaffinity=1)`。
- `phase18_floating_contact.xml`：显式 Newton/pyramidal/condim=3/contact 参数及 `(floor=1/0)`。
- `phase18_wheel_contact_probe.xml`：左右真实 wheel mesh、三向 carriage、wheel torque probe。
- `phase18_nominal.json`：timing、contact、case matrix、wheel radius 和 frozen thresholds。
- `run_mujoco_contact_floating_base.py`：2 ms normal/rolling/lateral/friction/free-flight/touchdown/reset runner，逐步 CSV、summary、manifest 和 non-overwrite。
- 方法、grounding、reuse contract、formal-v4 evidence 与历史回归记录。

## Key Results

- 20/20 formal gates PASS，11 个 CSV 跨进程 exact。
- wheel-only active set 正确，initial/unexpected contact 均为 0。
- static load relative error `9.85e-05`，vertical impulse error `2.15e-04 N·s`。
- positive/negative wheel torque 分别产生 `+X/-X` 位移；frictionless rolling displacement 近零。
- lateral `mu=0` 保持速度，`mu=1/2` 在正负方向均衰减到数值零。
- full-model zero-command touchdown：first contact `52 ms`，penetration `3.24 mm`，closure residual `0.115 mm`，free-fall acceleration error `8.13e-04 m/s²`。
- Phase 02/04/14/15/16/17 回归全部 PASS；19 个 C++/ROS tests 零失败。

## Decisions

- Phase 18 不使用腿控制；完整模型 authority 仅为 `0.2 s` free-flight/touchdown window。
- contact force/slip 是 validation-only diagnostics，不扩张 `RobotState`/ROS schema。
- Phase 18 使用专用 Python MuJoCo physics runner；Phase 16/17 C++ controller loop 不修改。
- `formal-v5` 是正式 authority；此前 formal runs 保留为非覆盖演进记录。

## Evidence

- [Formal summary](evidence/automated/2026-08-25-formal-v5/phase18_validation.json)
- [Run manifest](evidence/automated/2026-08-25-formal-v5/run_manifest.json)
- [Automated validation](evidence/automated/2026-08-25-validation.md)
- [Grounding](evidence/grounding.md)
- [Reuse contract](evidence/reuse_contract.md)
- [Method](../../../experiments/mujoco_contact_floating_base_validation.md)
- [Review](REVIEW.md)

## Reproduction

```bash
cd /home/t/W_L_ws
./.venv/bin/python tools/experiments/run_mujoco_contact_floating_base.py \
  --output-dir data/experiments/<new-phase18-run-id>/raw
```

必须使用新的空目录；已存在的正式 evidence 不可作为重跑目标。

## Limits and Next Use

- `mu=1`、soft contact 和 solver 参数只是 nominal profile，未由真机识别。
- `condim=3` 不验证 rolling/torsional resistance。
- Phase 19 可以在本 contact plant 上开始 z/pitch/leg posture/wheel position 的简单站立；不得把本 Phase 的短时触地写成站立 PASS。
- 未来 SolidWorks/identified revision 使用同一 runner/new config/new run 重跑，不覆盖本 RECORD。
