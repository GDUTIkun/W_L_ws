# Phase 17: nominal Joint PD 与重力补偿 — REVIEW

Verdict: `PASS`

## Reviewed Scope

- PLAN Goal、Scope、Frozen Decisions、DG01–DG08、T01–T10 和 Acceptance Criteria。
- Controller Core mode/reference/PD/gravity/clamp/diagnostics，ROS 静态 profile，Phase 16 runner 兼容扩展，Phase 17 config/wrapper/method。
- 最终 `2026-08-25-formal-v3` 的 25 份逐 tick CSV、validation summary、manifest 和全部 hash。
- Jazzy/Core/ROS/Adapter tests、coordinate、Phase 14/15 与最新 Phase 16 zero/fault regression。

## Goal and Gate Review

| Area | Actual evidence | Result |
| --- | --- | --- |
| Safe opt-in boundary | 默认 zero 保持；非法 mode/vector/limit/reference 拒绝；显式 ROS profile 发布有限非零 torque 且保留 source time | PASS |
| Controller law | measured-state PD、解析 gravity、求和后逐关节对称 clamp；reset 恢复初始 reference；diagnostics 完整 | PASS |
| Gravity model | 每腿 3 个解析谐波；150 姿态 reduced-bias 最差 `5.27e-12 N·m`，势能梯度最差 `6.78e-9 N·m`，C++/JSON 差 `0` | PASS |
| Wheel eccentricity | wheel gravity 最大 `4.07e-4 N·m`，未理想化清零 | PASS |
| Hold/comparison | PD+G 最终最大误差 `0.002831 rad`，PD-only `0.115862 rad`；短时 gravity-only 优于 zero | PASS |
| Step matrix | 左右六关节正负阶跃全部通过；最差 settling `0.89 s`，最差 overshoot `0.0307`，最差 velocity `1.804 rad/s` | PASS |
| Symmetry/disturbance | 三类 symmetry 最差 `8.14e-5 rad`；三类 disturbance 最终恢复误差小于 `0.002832 rad` | PASS |
| Saturation/replay | clamp 不越 profile limits；ZOH 差 `0`；reset 与 fresh-process replay exact | PASS |
| Reuse/non-overwrite | JSON/YAML 可注入 offset/coefficients；formal→v2→v3 均新目录保留；9 inputs/25 outputs hash 最终复核匹配 | PASS |
| Regressions | 19 tests；coordinate、Phase 14/15、最新 Phase 16 的 24/24 gates 全 PASS | PASS |

## Findings

### Blocking

None.

### Resolved During Review

- 初始 wheel gains 的正式前探索 run settling 为 `1.05–1.07 s`，未通过冻结的 `1.0 s` gate。未放宽阈值；改为 `Kp=0.3, Kd=0.05` 后在新正式目录达到 `0.88–0.89 s`。
- 初版 runner/ROS 只能选择内置 current nominal profile，无法充分满足后续 SolidWorks revision 复用。已增加 JSON/YAML offset 与谐波系数注入，使用新 `formal-v3` 重跑；旧正式目录未覆盖。
- v2 增加了 C++ tick gravity 与 JSON profile 的逐点交叉检查，避免只验证 Python 侧系数。

### Non-blocking / Accepted Limits

- gravity-only 无 PD/阻尼时不是渐近稳定控制器；其证据只用于短时模型补偿对照，闭环稳定结论来自 PD+gravity。
- 谐波 basis `[q_h, q_h+q_k, q_h+q_k+q_w]` 适用于当前平面拓扑。几何/拓扑变化后必须重新推导 profile 并重跑 oracle，不能继承系数。
- gains 和 `[6,6,1] N·m` limits 只属于 current nominal ideal-actuator 仿真，不是电机或真机安全参数。
- fixed-base identity pose、contact-disabled 范围没有验证 base orientation gravity、轮地接触、floating-base 或站立。

## Decision Gate Closure

- DG01–DG03：Joint-torque PD+gravity、离线 oracle、Core 内部分段常值 reference 均按冻结设计实现。
- DG04：gravity provenance/profile/sign/branch 由双 oracle、C++ profile 交叉检查和 manifest 关闭。
- DG05：探索调参冻结为 hip/knee `12/1.5`、wheel `0.3/0.05`，limits 每侧 `6/6/1 N·m`。
- DG06：hold/step/symmetry/saturation/disturbance/replay 所有正式 gate 通过。
- DG07：default zero、Phase 16 fault/replay、coordinate、Phase 14/15 和 package tests 全部回归 PASS。
- DG08：所有文档与 summary 均限定 simulation-only；真机/contact/floating-base gate 保持开放。

## Review Conclusion

Phase 17 在 current nominal、fixed-base、contact-disabled simulation-only 范围达到目标，没有 blocking finding。允许创建 RECORD 并把 ROADMAP 更新为 `complete`。本 PASS 不允许直接上真机，也不证明接触、floating-base、站立或真实执行器安全性。
