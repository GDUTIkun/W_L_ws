# Phase 18: nominal 轮地接触与 floating-base plant 验证 — REVIEW

Verdict: `PASS`

## Scope Review

- wheel-only collision mask：PASS。compiled active geoms 仅为 floor 和两个命名 wheel geom，reset contact 为 0，未批准 pair 为 0。
- actual-wheel probe：PASS。左右真实 mesh 的 normal、正负 rolling、正负 lateral 和 `mu=0/1/2` matrix 均通过。
- contact wrench：PASS。world-FLU 符号由静态重量和冲量—动量独立 oracle 支持；friction cone 与切向功率 gate 通过。
- full-model floating plant：PASS。六路零力矩、base weld 关闭的 free-flight/touchdown 在冻结 `0.2 s` 窗口内通过 gravity、contact、base pose/twist/quaternion、COM、closure、finite 和 reset replay。
- compatibility/reuse：PASS。Phase 02/04/14/15/16/17 全部回归；fresh-process 11 CSV exact；non-overwrite 生效。
- scope boundary：PASS。没有真机、站立、控制器调参、公共 message 或 calibrated contact claim。

## Evidence Review

正式 authority：[`formal-v5`](evidence/automated/2026-08-25-formal-v5/phase18_validation.json)，20/20 checks PASS。命令和关键指标见 [自动验证记录](evidence/automated/2026-08-25-validation.md)。

| Gate | Result | Evidence |
| --- | --- | --- |
| Collision eligibility | PASS | 3 active named geoms，0 initial/unexpected contact |
| Normal scale/sign | PASS | load relative error `9.85e-05`；impulse error `2.15e-04 N·s` |
| Rolling | PASS | positive `+X`、negative `-X`；frictionless displacement `<5e-05 m` |
| Lateral/friction | PASS | `mu=0` 保持 `0.2999 m/s`；`mu=1/2` 双向衰减到数值零 |
| Numerical contact | PASS | probe/full penetration `<0.004 m`；friction power `<0.001 W` |
| Floating touchdown | PASS | contact step 26；gravity error `8.13e-04 m/s²`；closure `0.115 mm` |
| Determinism | PASS | same-process replay + fresh-process 11 CSV exact |
| Historical regression | PASS | 19 C++/ROS tests + Phase 02/14/15/16/17 |

## Findings

Blocking findings: None.

Non-blocking limits:

- nominal `condim=3` 只验证法向与二维 sliding friction；真实 rolling resistance、torsional friction、轮胎 compliance 仍需 MuJoCo–real contact identification。
- zero-control full model 的 authority 只到首次触地后的 `0.2 s`。探索运行显示长期无腿控制会倒塌并产生更深 wheel penetration；这是 Phase 19 必须加入站立控制的预期边界，不是本 Phase 的站立证据。
- Phase 18 使用专用 MuJoCo Python plant runner，而未扩展 Phase 16 C++ controller loop。该受控变更减少共享代码改动，且 Phase 16/17 完整回归 PASS。

## Conclusion

Phase 18 的 current nominal simulation-only 目标已完成。接触参数没有真实标定含义；下一控制层可以使用本 profile 开始简单站立，但必须保留本 Phase 的 collision/contact gates 和非覆盖运行方式。
