# Phase 19: exact 2D sagittal 简单站立 — REVIEW

Verdict: `PASS`

## Scope Review

- exact-planar plant：PASS。source→derived generator 只把 base freejoint 替换为 world X/Z/+Y pitch，preserved compiled physics 差为 `0`。
- equilibrium/state contract：PASS。zero-wheel-torque compliant equilibrium、双轮载荷、closure、qacc、site/quaternion/Jacobian 与 wheel sign 已通过并 exact replay。
- sampled controller：PASS。v2 失败已归因为 contact plant 上 `12/1.5` leg PD 的 10 ms ZOH 不稳定；standing profile `8/1` 保持原 timing，pre-freeze full nonlinear envelope PASS。
- Core/runtime：PASS。`kSimpleStanding` 显式 opt-in；fixed leg reference + support + sampled PD、equal common wheel feedback、首帧双轮接触、timing/contact/state/envelope/torque fail-closed latch 和 reset 已实现。
- formal/reuse：PASS。11 个 10 s normal/perturbation cases、4 个双 episode fault cases、fresh replay、non-overwrite、historical regressions 和 revision-workflow fresh namespace 全通过。
- scope boundary：PASS。没有真机、3D standing、WBC/QP/NMPC、公共 message 修改或 calibrated contact claim。

## Evidence Review

正式 authority：[`formal-v4`](evidence/automated/2026-08-26-formal-v4/summary.json)，完整说明见 [自动验证记录](evidence/automated/2026-08-26-validation.md)。

| Gate | Result | Evidence |
| --- | --- | --- |
| Pre-freeze attribution | PASS | `12/1.5` failure reproduced；`8/1` nonlinear 10 s envelope PASS |
| Local model | PASS | rank `4`；spectral radius `0.984789`；A/B step convergence `7.05e-8` |
| Normal/initial recovery | PASS | nominal、`±0.005 rad` pitch、`±0.01 m/s` rolling |
| Disturbance recovery | PASS | `±0.2 N × 0.1 s`、`±0.02 N·m × 0.1 s`、leg `±0.002 rad` |
| Safety/reset | PASS | contact/invalid/nonmonotonic/saturation 均零输出锁存；2-episode exact reset |
| Runtime invariants | PASS | bilateral contact 100%；equal wheel/ZOH/Adapter sign error 均 `0` |
| Determinism | PASS | formal-v4 primary/replay summary SHA-256 exact |
| Compatibility | PASS | 19 tests；Phase 02/14/15/16/17/18 全部回归 |
| Reuse/non-overwrite | PASS | fresh namespace pipeline PASS；非空 formal 目录在仿真前拒绝 |

## Findings

Blocking findings: None.

Non-blocking limits:

- authority 仅为 current nominal exact-planar simulation。base 的 Y/roll/yaw 已从模型删除，不能推断完整 3D 或真机能站立。
- 正式 pitch 初态冻结为 contact-projected `±0.005 rad`。旧 `±0.01 rad` fixed-leg reset 离开 bilateral-contact manifold，只保留探索证据。
- frozen 外力/力矩属于小扰动验证，不声明大扰动恢复、region of attraction、跌倒恢复或单轮支撑。
- raw 26-state generalized-coordinate finite differences 离开 equality/contact manifold 且不随步长收敛；其“极点”不是本 Phase 的物理 authority。release 依据是 admissible local model convergence 加完整 nonlinear plant formal。
- 现有 Graphify 图缺少 Phase 19 最新闭环；Codex 未执行 extract/update，增量维护 prompt 已保存于 [`graphify_incremental_prompt.md`](evidence/graphify_incremental_prompt.md)。

## Conclusion

Phase 19 的 current nominal exact-planar simulation-only 简单站立目标已完成，可以进入独立的完整 3D standing Phase。下一阶段必须增加 roll/yaw/lateral sensing 与 control authority，不能把本 Phase 的二维 PASS 直接迁移为 3D 或真机 PASS。
