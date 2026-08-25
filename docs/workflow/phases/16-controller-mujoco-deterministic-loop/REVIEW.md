# Phase 16: Controller ↔ MuJoCo 确定性闭环运行基线 — REVIEW

Verdict: `PASS`

## Reviewed Scope

- PLAN 的 Goal、Scope、Frozen Decisions、DG01–DG06、T01–T08 和 Acceptance Criteria。
- `phase16_contact_free.xml`、`phase16_nominal.json`、C++ deterministic executable、Python wrapper 和 ROS regression hardening。
- 最终 `2026-08-25-nominal-v2` 的 3 份逐 tick CSV、validation summary、run manifest 及全部 hash。
- coordinate、Core/ROS/Adapter tests、ROS topic/reset smoke 和 Phase 14/15 新目录回归。

## Goal and Gate Review

| Area | Evidence | Result |
| --- | --- | --- |
| Scene/timing | MuJoCo 3.7.0 加载；`nu=6`、`neq=3`、命名 `floor`；2 ms physics、10 ms control、5-step ZOH | PASS |
| Tick execution | 2 episode × 100 ticks；physics begin/end、source/receipt time、Controller `dt` 全部符合冻结步序 | PASS |
| Current Core behavior | Core torque 与 native ctrl 最大绝对值均为 `0 N·m`；全部字段有限 | PASS |
| ZOH/replay | ZOH 最大差 `0`；reset/fresh 与 fresh/fresh 最大数值差均为 `0`；fresh CSV hash 相同 | PASS |
| Lifecycle/fail-safe | duplicate/future/stale/timeout/reset-old 拒绝或归零；timeout 后与新 epoch 恢复 | PASS |
| Manifest/non-overwrite | 8 个输入与 3 个输出 SHA-256 最终复核匹配；重复目录 exit 1 | PASS |
| Reuse | model/config 由路径选择，profile 名不再锁定 nominal；Controller rebuild/plant revision 无需修改循环算法 | PASS |
| Regressions | 18 ROS/Core/Adapter tests；coordinate、Phase 14、Phase 15、独立 ROS smoke 全 PASS | PASS |

## Findings

### Blocking

None.

### Resolved During Review

- Python CLI 原先只允许 `--profile nominal`，会妨碍后续 identified profile 使用自身名称。已移除该枚举限制，并在新 `2026-08-25-nominal-v2` 目录重新生成全部正式证据；旧目录未覆盖。
- ROS pub/sub test 原先处于默认 Domain，会接收用户另一个终端中的持续消息。测试改为端点发现后单样本驱动，并固定 `ROS_DOMAIN_ID=232`；未停止用户进程，外部节点仍运行时测试通过。

### Non-blocking / Accepted Limits

- 当前 Core 没有 PD 或重力补偿，nominal torque 必须为零；fault runner 的 `1 N·m` 是明确分列的 Adapter watchdog 实验输入，不是 Controller 输出。
- fault 事件位置属于 schema v1 的冻结 schedule；未来 plant/Controller profile 可原样复用。若改变 fault schema，需升级配置/日志 schema，而不是静默改变含义。
- contact-disabled fixture 仍保留命名 floor 以满足 production Adapter 对象契约；本 Phase 没有验证轮地接触、摩擦或 floating-base 落地。
- bitwise replay 只对 manifest 中同一 executable、模型、config 和环境成立；任一输入变化后必须新建 run，并重新比较数值。
- ROS launch 受 wall-clock 调度影响，只是 transport/schema/reset smoke，不参与 deterministic verdict。

## Decision Gate Closure

- DG01 CLOSED：Phase 04 mapping/watchdog/reset 只作回归；新增证据严格限定为 fixed-step loop/log/replay。
- DG02 CLOSED：production Core 保持零输出，没有加入临时反馈控制模式。
- DG03 CLOSED：2 ms/10 ms/5-step 唯一步序已由真实 200-tick log 验证。
- DG04 CLOSED：fresh/fresh hash 相同，fresh/reset 和跨进程最大数值差均为 `0`。
- DG05 CLOSED：全部冻结 fault event 的拒绝、归零和恢复均由 `faults.csv` 支持。
- DG06 CLOSED：确定性以 C++ fixed-step runner 为正式来源，ROS 只作兼容性 smoke。

## Review Conclusion

Phase 16 在 simulation-only 范围内达到目标，没有 blocking finding。允许创建 RECORD 并将 ROADMAP 更新为 `complete`。本 PASS 不关闭 Joint PD、contact、实时性、Phase 05 或任何真机 gate。

