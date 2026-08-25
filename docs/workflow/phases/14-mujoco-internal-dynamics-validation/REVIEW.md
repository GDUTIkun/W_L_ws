# Phase 14: MuJoCo 运动学与内部动力学验证 — REVIEW

Status: `review`

## Review Scope

- PLAN：[`PLAN.md`](PLAN.md)
- 审查范围：Phase 14 fixture/config/script/evidence、`wheel_leg.xml` 精确结构角修正、Adapter offset 回归及相关文档更新
- 审查者与日期：Codex，2026-08-25

## Implementation Check

| PLAN Task | Delivered | Evidence | Result |
| --- | --- | --- | --- |
| T01 | 编译参数 grounding 与逐项 manifest | [`model_parameter_grounding.md`](evidence/model_parameter_grounding.md)、[`parameter_manifest.json`](evidence/automated/parameter_manifest.json) | PASS |
| T02 | 完整 contact-free fixture、五刚体闭链单腿 fixture、invariant checks | [`phase14_contact_free.xml`](../../../../simulation/mujoco/model/phase14_contact_free.xml)、[`phase14_single_leg.xml`](../../../../simulation/mujoco/model/phase14_single_leg.xml) | PASS |
| T03 | 独立 FK/Jacobian 参考与 7 姿态 sweep | [`run_mujoco_internal_dynamics.py`](../../../../tools/experiments/run_mujoco_internal_dynamics.py)、[`phase14_validation.json`](evidence/automated/phase14_validation.json) | PASS |
| T04–T08 | gravity、M(q)、forward/inverse、constraint、coupling、energy、replay | [`2026-08-25-validation.md`](evidence/automated/2026-08-25-validation.md) | PASS |
| T09 | 后续共同辨识输入/状态/日志契约 | [`reuse_contract.md`](evidence/reuse_contract.md) | PASS |
| T10 | 正式方法、数据包、README、自动证据 | [实验方法](../../../experiments/mujoco_internal_dynamics_validation.md)、[data README](../../../../data/experiments/2026-08-25-mujoco-internal-dynamics/README.md) | PASS |

## Validation Results

| Validation | Command / Procedure | Actual Result | Evidence |
| --- | --- | --- | --- |
| Phase 14 sweep | `./.venv/bin/python tools/experiments/run_mujoco_internal_dynamics.py` | 9/9 groups PASS | [`phase14_validation.json`](evidence/automated/phase14_validation.json) |
| Coordinate contract | `./.venv/bin/python tools/maintenance/test_mujoco_coordinate_contract.py` | PASS | [`2026-08-25-validation.md`](evidence/automated/2026-08-25-validation.md) |
| ROS build | `colcon build --symlink-install --packages-up-to wheel_leg_mujoco` | 4 packages finished | [`2026-08-25-validation.md`](evidence/automated/2026-08-25-validation.md) |
| Adapter regression | `colcon test --packages-select wheel_leg_mujoco --event-handlers console_direct+` | 6/6 gtest PASS | [`2026-08-25-validation.md`](evidence/automated/2026-08-25-validation.md) |
| Test results | `colcon test-result --verbose` | 18 tests, 0 errors, 0 failures, 0 skipped | [`2026-08-25-validation.md`](evidence/automated/2026-08-25-validation.md) |
| Repository checks | `git diff --check`; Python `py_compile`; JSON/CSV schema check | PASS；CSV 251 rows、16 columns、row width consistent | [`2026-08-25-validation.md`](evidence/automated/2026-08-25-validation.md) |

## Findings

### Blocking

None.

### Non-blocking

- Mass/COM/inertia are imported/compiled nominal values; real accuracy is deliberately unresolved and transferred to later MuJoCo–real identification.
- Joint damping, frictionloss and armature remain zero MuJoCo defaults with unknown real values.
- Energy/replay use zero gravity to isolate actuator/inertia numerical work balance; gravity and full closure are separately validated and cannot be inferred from that replay alone.
- Contact fidelity remains out of scope; the formal fixtures intentionally disable contact.

## Decision and Evidence Review

- 冻结决策是否被保持：是。MuJoCo 版本、canonical frames/order/sign、Adapter 边界和 no-hardware gate 未改变；模型修正只把应为精确结构角的截断常量恢复为 `pi/2`/`pi`，并同步重算模型 offset。
- 证据是否足以支持技术结论：是。所有结论来自真实 MuJoCo 计算、逐项最差值、版本化输入和可重复输出；没有从成功构建或零输出推断动力学 PASS。
- 是否存在需要新 Phase 的开放问题：真实 mass/COM/inertia、passive 参数、actuator 和 contact fidelity 均由现有后续 MuJoCo–真机共同辨识路线承担；本 Phase 无新增 blocking Phase。

## Verdict

`PASS`

Phase 14 仅通过 `MuJoCo internally consistent`。它不通过、也不声称 `MuJoCo matches the real robot`。
