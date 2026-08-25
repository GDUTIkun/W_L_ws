# Phase 15: MuJoCo 完整闭链运动学与 Jacobian 验证 — RECORD

Status: `complete`

## Delivered Outcome

current nominal MuJoCo 双腿五刚体闭链已经在冻结工作域内完成可重复的被动装配、独立 FK、轮心/名义接触点、约束降维 Jacobian、中心有限差分、速度、虚功、方向和左右对称验证。正式结果 `overall_pass=true`，不包含真机操作或真机一致性声明。

## Frozen Technical Record

- 独立坐标为每侧 `[hip, knee, wheel]`，被动坐标为 `[connect1, connect2]`；按命名对象解析地址。
- nominal 分支左腿为 `[knee,-knee]`，右腿为 `[-knee,-knee]`，Newton/least-squares continuation 与正反路径独立关系一致。
- closure 使用 site 三维位置残差，数值有效秩为 2；`S=[I;-pinv(Jp)Ja]`，`J_reduced=J_object·S`。
- nominal wheel radius 为 `0.05 m`，来自当前 compiled mesh 约 `0.10 m` 径向直径；不采用 Simulink `0.08 m`，也不宣称实物标定。
- 正 wheel 角速度使底部材料点沿 `-X`，无滑轮心滚动方向为 `+X`。
- 左右位置关于 XZ 面镜像；左右 wheel body frame 使用相同右手轴。
- output/profile/runner 使用 manifest+SHA-256；非空目录拒绝覆盖，未来 revision/identified profile 新增 run。

## Evidence

- [自动验证记录](evidence/automated/2026-08-25-validation.md)
- [正式结果](evidence/automated/2026-08-25-nominal/phase15_validation.json)
- [几何 manifest](evidence/automated/2026-08-25-nominal/geometry_manifest.json)
- [运行 manifest](evidence/automated/2026-08-25-nominal/run_manifest.json)
- [完整工作域 CSV](evidence/automated/2026-08-25-nominal/workspace.csv)
- [几何 grounding](evidence/geometry_grounding.md)
- [复用与非覆盖契约](evidence/reuse_contract.md)
- [验证方法](../../../experiments/mujoco_closed_chain_kinematics_validation.md)
- [REVIEW](REVIEW.md)

## Reproduction

```bash
cd /home/t/W_L_ws
./.venv/bin/python tools/maintenance/test_mujoco_coordinate_contract.py
./.venv/bin/python tools/experiments/run_mujoco_internal_dynamics.py \
  --output-dir data/experiments/<new-phase14-regression-run-id>/raw
./.venv/bin/python tools/experiments/run_mujoco_closed_chain_kinematics.py \
  --output-dir data/experiments/<new-phase15-run-id>/raw
```

已存在的正式 evidence 目录不可作为重跑目标。

## Limits and Next Use

- PASS 仅针对 current nominal MuJoCo geometry/profile 和冻结工作域。
- SolidWorks 尺寸变化后生成新 model/config/run，使用同一 runner 重跑，不修改本 RECORD。
- 真机恢复后，identified profile 同样从本 Phase 入口重跑，再做 nominal/identified/real comparison。
- contact dynamics、摩擦/滑移、floating-base 落地和控制复杂度必须在后续独立 Phase 验证。

