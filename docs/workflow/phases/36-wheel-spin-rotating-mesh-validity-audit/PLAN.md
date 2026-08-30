# Phase 36 PLAN — Wheel-Spin / Rotating-Mesh Validity Audit

状态：`complete`  
日期：2026-08-30

## 目标与范围

唯一问题是 current nominal MuJoCo/WBC 中 canonical absolute wheel spin 的 `±1 rad`
是否为必要模型有效域。只做离线 fixed-state/phase-isolated 审计；不修改或绕过
`NominalWbcModel::kOutsideWorkspace`，不改 controller、NMPC、planner、QP、mesh、摩擦或物理参数。

## 冻结决策

- canonical wheel delta 与 native hinge phase 的符号为 `q_canonical=-q_native`，equilibrium wheel phase 为零。
- 固定 Phase35 H0 tick 0 的 base/leg/passive state 与 actuator torque，只改变 wheel hinge phase。
- sweep、periodic pairs、finite-symmetry detection orders 与阈值在 formal 结果生成前固化于
  `simulation/mujoco/config/phase36_wheel_phase_validity_v1.json`。
- `±1 rad` 界外 live WBC 按契约拒绝；不得绕过并伪造界外 QP/realized-wrench/torque。
  界外使用同一 MuJoCo full-body plant 的瞬时 dynamics oracle；QP 项明确记为 unavailable-by-contract。
- `q` 与 `q+2π` 是强 physical-equivalence oracle。接触关闭对照仅用于区分 raw collision contact
  与惯性/实现来源，不代表生产配置。
- 分类只允许 P36-A/B/C/D/E/U；本 Phase 不实施 repair。

## 任务

| ID | 任务 | 验收 |
| --- | --- | --- |
| P36-T01 | 冻结 wheel/native/mesh/contact/WBC 语义 | complete |
| P36-T02 | 实现并运行固定状态 coarse、boundary、periodic sweep | complete |
| P36-T03 | 比较 geometry、M/h/J、Axi、closure 与 contact-off 对照 | complete |
| P36-T04 | 相同 torque 的 instantaneous response oracle | complete |
| P36-T05 | 独立判定 `±1 rad` boundary specificity | complete |
| P36-T06 | fresh replay、REVIEW；仅 PASS 后 RECORD/ROADMAP | complete |

## 验证入口

使用 `./.venv/bin/python` 先做依赖探针及 `py_compile`，再运行：

```bash
./.venv/bin/python tools/experiments/run_phase36_wheel_phase_validity.py --output <new-dir>
```

第二次以新目录运行并传入 `--replay-of`。两次 summary、classification 和数值摘要必须一致。

## 停止条件

语义无法确认、formal 数据缺失、periodic oracle 不可解释或两次运行不一致时 REVIEW=`REWORK`、P36-U；
不得用 controller repair 扩张本 Phase。
