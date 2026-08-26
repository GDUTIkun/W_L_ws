# Phase 19 v1: nominal 平面简单站立（固定腿姿态 + 轮式平衡）— REVIEW

Status: `review`

Verdict: `REWORK`

## Review Scope

- PLAN：[`PLAN-v1-2026-08-26-REWORK.md`](PLAN-v1-2026-08-26-REWORK.md)
- 审查范围：Phase 19 pre-freeze grounding、scene/profile、四状态局部模型、10 s exploratory full-plant cases、fresh-process replay 和 non-overwrite
- 审查者与日期：Codex，2026-08-26
- 未审查/未实现：Controller Core standing mode、C++ deterministic standing loop、formal disturbance/fail-safe matrix、完整历史回归和真机

## Implementation Check

| PLAN Task | Delivered | Evidence | Result |
| --- | --- | --- | --- |
| T01 | Core/Adapter/runner、Phase 15–18、Simulink 边界 grounding；Graphify 仅查询 | PLAN Current State / Grounding / Execution Notes | PASS |
| T02 | Phase 19 scene 与 exploratory candidate profile；未得到冻结定义的 equilibrium | [`phase19_standing.xml`](../../../../simulation/mujoco/model/phase19_standing.xml)、[`phase19_exploration.json`](../../../../simulation/mujoco/config/phase19_exploration.json) | FAIL |
| T03 | site pose/Jacobian twist/quaternion pitch/reset anchor 已进入 evaluator；wheel rolling/Core 端到端未进入 | [`run_mujoco_simple_standing_exploration.py`](../../../../tools/experiments/run_mujoco_simple_standing_exploration.py) | PARTIAL |
| T04 | 仅探索固定腿 native support+PD；按 gate 未移入 Core | pre-freeze timeseries | FAIL |
| T05 | 4-state/1-input model、rank、poles、affine drift 和 10 s plant cases | [`summary.json`](evidence/exploratory/2026-08-26-prefreeze/summary.json) | FAIL |
| T06–T10 | PLAN 要求 DG04 失败时不得继续 | 未进入 | BLOCKED |
| T11 | 真实失败 evidence、replay、PLAN/ROADMAP/REVIEW 一致化 | [验证记录](evidence/exploratory/2026-08-26-validation.md) | PASS |

## Validation Results

| Validation | Actual Result | Evidence |
| --- | --- | --- |
| 10 ms local model | controllability rank `4`，但 spectral radius `1.0320567`，FAIL | [`summary.json`](evidence/exploratory/2026-08-26-prefreeze/summary.json) |
| equilibrium drift | one-tick pitch-rate drift `0.0863837 rad/s`，DG02 未关闭 | 同上 |
| unconstrained full 3D 10 s | final sagittal values碰巧接近门槛，但 lateral `0.0544 m`、roll/yaw `0.1312 rad` 超限 | 同上 |
| diagnostic planar stabilization 3 cases | 全部 finite、双轮几乎持续接触，但 final x/pitch 和 roll/yaw 仍超限 | 同上 |
| fresh-process replay | primary/replay timeseries 与 summary SHA-256 exact | [验证记录](evidence/exploratory/2026-08-26-validation.md) |
| non-overwrite | 非空目录在仿真前拒绝 | 同上 |
| coordinate contract | PASS | 同上 |
| Phase 18 scene compatibility | compiled dimensions/timestep exact | 同上 |
| Python/static hygiene | `py_compile`、`git diff --check` PASS | 同上 |

## Findings

### Blocking

1. **B01 — 冻结 equilibrium 不成立。** 当前候选的 10 ms 仿射漂移非零，特别是 pitch-rate 增量 `0.08638 rad/s`；不能把固定腿 support torque 和双轮接触等同于 upright、零轮扭矩静态平衡。
2. **B02 — DG04 明确失败。** 四状态模型虽然可控，但候选 gain 的闭环谱半径大于 1；10 秒未倒只证明有限时间有界，不能证明收敛或局部稳定。
3. **B03 — Scope 内部冲突。** 完整 3D plant 的 lateral/roll/yaw 是自由模态，当前 controller 又被限定为 sagittal/common-wheel，无法同时承诺这些泄漏硬门槛。诊断性平面外力不属于原冻结 scope，也没有修复 sagittal recovery。
4. **B04 — Formal authority 缺失。** 根据 PLAN 的 pre-freeze gate，T06–T10 正确地没有执行；因此没有 C++ Core/Adapter/ZOH/fail-safe/formal evidence，不能创建 RECORD 或标记 complete。

### Non-blocking

- exploratory runner、profile、raw evidence 和 replay 都是新增路径，没有覆盖 Phase 14–18 formal evidence。
- Phase 19 scene 保持 Phase 18 wheel-only contact、solver 和 compiled topology；当前失败不是由意外 collision profile 改动造成。

## Decision and Evidence Review

- 冻结决策是否被保持：实现 gate 被保持；没有用 Graphify update、真机、WBC、积分器、阈值放宽或诊断性外力伪造 PASS。原“完整 3D plant + 只控 sagittal + 对 3D 泄漏设硬门槛”决策本身需要重划。
- 证据是否足以支持技术结论：足以支持 `REWORK`，不足以支持 standing PASS 或“该结构永远不可能稳定”的穷尽性结论。
- 下一步技术决策：优先把 Phase 19 重划为显式受约束、可审计的 2D sagittal validation plant；完整 3D standing 单独增加 roll/yaw/lateral control 层。若坚持当前完整 3D authority，则必须扩大 controller scope，不能继续称为 common-wheel 四状态简单站立。

## Verdict

`REWORK`

不创建 `RECORD.md`。重做前必须先关闭 B03 的 plant/controller 边界选择，再从 DG02 equilibrium 和 DG04 gain 开始，保留本轮失败 evidence 不覆盖。
