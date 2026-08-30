# Phase 45 Review

结论：`REWORK`  
日期：2026-08-30

## Findings

1. **BLOCKING — DG45-EQ FAIL.** right actual `ddxi=-0.0533965 m/s2`，超过冻结
   `0.05 m/s2`；left为`-0.0103356`。按stop order未进入AUTH/REAL/SHORT/10 s/REAUDIT。
2. **PASS — baseline provenance.** 两次no-repair均tick111首次right contact loss，semantic error=0。
3. **PASS — actual-contact oracle.** controller与独立Phase44-convention slip/map/bias max error均0；
   双侧contact/load/geometry有效并active。
4. **PASS — reproducibility.** formal-v3与fresh replay-v2 machine-readable semantic error=0；v3还保留
   既有wrench/slack task index，修复v2的diagnostic enum兼容性问题。
5. **REJECTED evidence — formal-v1.** runner曾在EQ FAIL后继续AUTH，违反PLAN；目录append-only保留但
   不参与结论，v2修复为严格停止。
6. **Scope preserved.** `nmpc_12d_run=false`、`planner_run=false`、`phase34_tracking=false`、
   `repair_16d=false`、plant/contact/friction modification=false、wrench tuning=false、solver change=false、
   `new_candidate_count=1`。

## Verification

- `.venv` dependency probe：MuJoCo 3.7.0、NumPy 2.2.6、SciPy 1.15.3；`py_compile` PASS；
- targeted `colcon build` PASS；core 17/17、adapter 6/6，workspace aggregate 35 tests、0 failure；
- formal-v3/replay-v2共28 CSV + 14 JSON可解析，numeric non-finite=0；
- `git diff --check` PASS；fresh replay max semantic error=0。

## Verdict

`REWORK / P45 structure FAIL at DG45-EQ`。不创建RECORD，不把ROADMAP标为complete，不进入Phase46。

## REWORK Addendum — Equilibrium Compatibility

tick0-only 审计 PASS，结论为 `FIXED_WRENCH_EQUILIBRIUM_MISMATCH`：fixed case 的 right
`desired=0 -> QP=3.747e-11 -> MuJoCo=-0.0533965 m/s2`；right actual 分解几乎全部来自
leg/non-wheel `-0.05339650936 m/s2`，base/wheel/Jdotv 可忽略。只改变既有
`left/right Fx, Ty` 请求 `[-0.0631927, -0.187486, +0.00259355, +0.00279667]`
后，两侧 actual `ddxi=0, a_t=0` 的最大残差为 `4.06e-14 m/s2`，且 hard/slack/torque、
whole dynamics/contact closure 全部通过；fresh replay semantic error=0。

最终 authority 为 formal-v2/replay-v2。formal-v1/replay-v1 数值相同，但汇总与归档 probe
之间经过一次确定性重跑，因 provenance 不够直接而 rejected、append-only 保留。

REWORK 验证使用 `.venv`：MuJoCo 3.7.0、NumPy 2.2.6、SciPy 1.15.3；`py_compile`、
targeted `colcon build`、core 17/17 与 adapter 6/6 均 PASS，workspace aggregate 为35 tests、
0 failure。formal-v2/replay-v2 共14 CSV + 8 JSON、non-finite=0；四个 native probe 都只有
一条`pre_command`和一条`post_command`，无`stepped`记录，确认没有 trajectory integration。

因此 fixed wrench 下的 QP->MuJoCo gap 是 equilibrium incompatibility 的表现；没有 fixed-case
QP xi task realization mismatch 的证据。该 addendum 只关闭归因问题，不使原 DG45-EQ 变为 PASS，
也不授权 AUTH/rollout/Phase46。详细冻结口径和证据见 [REWORK.md](REWORK.md)。

## Compatible-H0 Continuation Addendum

结论仍为 `REWORK`。formal-v2 compatible wrench 已原值冻结，DG45-EQ 由原 FAIL 变为 PASS；
但 DG45-AUTH common channel 在 scale `1/0.5/0.25` 均为
`G_QP=+0.998203` 对 `G_MJ=-1.875899`，sign gate FAIL。最大 scale-convergence relative
仅 `1.43e-11`，该反号是可信结构 finding。differential channel 为
`+0.995035/+0.237023`，单独 PASS，不足以放行 unified AUTH。

按 mandatory stop，REAL、SHORT、10 s、post-repair reaudit 未运行；不创建 RECORD，不进入下一
Phase。formal-v1/fresh replay-v1 semantic error=0；两包共64 CSV + 10 JSON、non-finite=0。
`.venv` dependency probe、`py_compile`、targeted build、core 17/17、adapter 6/6、workspace
aggregate 35 tests/0 failure 与 `git diff --check` 均 PASS。完整审查见
[CONTINUATION.md](CONTINUATION.md)。
