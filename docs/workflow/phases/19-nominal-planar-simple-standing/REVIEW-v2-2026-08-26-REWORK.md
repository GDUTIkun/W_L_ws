# Phase 19 v2 pre-freeze: exact 2D sagittal 简单站立 — REVIEW

Status: `review`

Verdict: `REWORK`

## Review Scope

- PLAN：[`PLAN.md`](PLAN.md)
- 审查范围：v1 非覆盖归档、exact-planar generator/diff、柔性闭链/接触 equilibrium、state/sign/rolling contract、四状态与完整 26-state pre-freeze gate
- 未实现：Controller Core standing mode、C++ standing loop、formal envelope、历史 formal regression、真机
- 审查者与日期：Codex，2026-08-26

## Implementation Check

| Task | Result | Evidence |
| --- | --- | --- |
| P19V2-T01 | PASS | v1 PLAN/REVIEW/evidence 原路径保留，v2 PLAN/roadmap 非覆盖重划 |
| P19V2-T02 | PASS | [planar-model validation](evidence/automated/2026-08-26-planar-model/validation.md) |
| P19V2-T03 | PASS | [equilibrium validation](evidence/automated/2026-08-26-planar-equilibrium/validation.md) |
| P19V2-T04 | PASS | [state/sign validation](evidence/automated/2026-08-26-planar-state-contract/validation.md) |
| P19V2-T05 | FAIL | [full-plant pre-freeze](evidence/exploratory/2026-08-26-planar-prefreeze-v2/validation.md) |
| P19V2-T06–T09 | BLOCKED | PLAN 要求 DG19-05 通过后才能修改 Core/formal |
| P19V2-T10 | DONE | 本 REVIEW；Verdict=`REWORK`，不创建 RECORD |

## Findings

### Blocking

1. **B01 — 四状态 reset-local 模型不是当前 sampled plant 的充分状态。** 它给出 rank `4`、谱半径 `0.984789`，但每 tick 不重置闭链/contact/leg 隐藏状态的完整 `26×26` 闭环模型谱半径为 `1.767146`，含三个不稳定极点。
2. **B02 — nonlinear pre-freeze recovery 失败。** nominal 与 `±1e-5 rad` pitch 等五个 case 均未完成 `1 s` gate；pitch 超过 `0.5 rad` 且双轮接触显著丢失。因此不能冻结 gain/envelope，也不能进入 Core。
3. **B03 — 当前 controller scope 不足以解释或控制隐藏模态。** 需要先对完整 sampled eigenvectors 做 equality/contact/leg-mode 归因，再决定是修正 plant 数值配置、重新设计 leg sampled control，还是扩大可测状态/控制结构。不能只调四状态 LQR 或放宽阈值。

### Non-blocking

- derived exact-planar model fidelity、zero-wheel-torque equilibrium 和 canonical state/sign/rolling contract 已分别关闭 DG19-02/03/04，可在下一轮直接复用。
- current nominal equilibrium 的左右 active reference 小幅不同是模型真实不对称的结果，已被阈值约束并 exact replay；不是本次 REWORK 的原因。
- 所有 v1/v2 primary/replay evidence 均保留；没有 Graphify extract/update、真机操作、Core 写入或历史覆盖。

## Decision Review

现有证据足以否决“基于 reset-local 四状态模型直接实现 Core”的本轮候选，但不足以声称简单站立或所有四状态控制器从数学上永远不可能。下一轮应从完整 `26-state / 10 ms ZOH` 不稳定模态归因开始，并把“完整 sampled plant 稳定”置于 reduced-model gate 之前。

## Verdict

`REWORK`

DG19-05 保持 OPEN；P19V2-T06–T09 不执行，不创建 `RECORD.md`，ROADMAP 不标 complete。
