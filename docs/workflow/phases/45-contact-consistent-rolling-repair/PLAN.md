# Phase 45: Contact-Consistent Rolling Repair — PLAN

状态：`review`  
日期：2026-08-30

## 审核与冻结决策

Phase44 的 PLAN/REVIEW-addendum/RECORD/ROADMAP 一致闭合为 `P44-E`；初始 REVIEW=REWORK
保留为历史，不阻塞本 Phase。Phase45 只测试一个 hypothesis：同一个 WBC profile 内的 wheel-center
constant hold 与 actual-contact material-point rolling consistency 能否建立稳定 H0 rolling manifold。

实现不使用 analytic contact center 代替 Phase44 oracle。MuJoCo adapter-side experiment 在每个 control
tick 取第一个实际 wheel-floor contact，沿用 Phase44 的 geom-order normal、world +X tangent 投影、
`v_C + omega cross r` material-point convention，并把 reduced affine row、bias、slip 与 activation
作为一次 contact observation 交给 WBC。两侧各有 xi row 和 rolling row，但它们属于一个 profile、
一个 repair、一个 gate chain。

唯一 nominal gain 固定为 3.5 Hz：`Kxi_p=483.61061565337855 s^-2`、
`Kxi_d=43.982297150257104 s^-1`、`Kslip=21.991148575128552 s^-1`；不设 low/high，
不在结果后追加 gain。task scale 固定 1 m/s2。

contact activation 固定为 `enable=5 N`、`disable=2 N`、re-enable 连续 2 ticks；tick0 若 contact、
load、geometry、finite 全部有效则同步启用。contact missing、load <=2 N、invalid tangent 或 non-finite
立即禁用；2--5 N 保持当前 hysteresis state。load 不是 objective。

## Scope

冻结 Model B、Phase27 Minimal WBC semantics、fixed interaction wrench、6D contact wrench、scene、
friction、torque limits、solver、initial state 与 10 ms/2 ms timing。12D NMPC、planner、Phase34
step/ramp、16D repair、plant/contact/friction 修改、wrench/weight tuning 均禁止。

## Tasks 与 gates

| ID | 内容 | PASS/停止条件 | 状态 |
| --- | --- | --- | --- |
| P45-T01 / DG45-BASE | no-repair provenance | 双运行首个 right contact loss=tick111；semantic parity | done |
| P45-T02 / ORACLE | controller/Phase44 convention | slip error <=1e-10 m/s；affine actual acceleration error <=1e-7 m/s2 | done |
| P45-T03 / DG45-EQ | tick0 no-integration | abs(ddxi)<=0.05、abs(material tangent acceleration)<=0.01；contact/load/base/native qdd全报告 | v1 failed；compatible-H0 PASS |
| P45-T04 / DG45-AUTH | fixed-state +/- directional audit | common/differential unified projected `sign(G_MJ)=sign(G_QP)` 且 abs(G_MJ)>=0.05；0.5/0.25 convergence<=0.05 | compatible-H0 failed |
| P45-T05 / DG45-REAL | physical decomposition | xi/slip/native/base/leg贡献与whole-vector/contact closure可信 | not entered |
| P45-T06 / DG45-SHORT | 223 ticks = 2.23 s | 前述 gates PASS 后才运行；bilateral contact、rate/slip/xi/full-body/WBC/wrench全PASS | not entered |
| P45-T07 / DG45-ROLL | 1000 ticks = 10 s | SHORT PASS 后运行；stop-on-first-independent-failure | not entered |
| P45-T08 / DG45-REAUDIT | repaired snapshots | 10 s PASS 后取 tick0/early/middle/late，按 Phase44 regime oracle复审 | not entered |
| P45-T09 | formal/replay/regression | dependency probe、py_compile、targeted build/test、parse/nonfinite、fresh replay、diff-check | done |
| P45-T10 | REVIEW/RECORD | blocking finding => REWORK；仅 REVIEW PASS 创建 RECORD/ROADMAP complete | REVIEW=REWORK |

AUTH 的唯一 input family 是同一 unified module 的 common/differential方向：同侧同时施加
`delta_xi=delta_slip=0.01 m/s2`。输出投影为同 mode 的 `ddxi` gain 与 material-point tangential
acceleration gain 的平均；这不是 xi-only/slip-only candidate。任一关键 self channel sign reversal
立即判 `P45 structure FAIL`，不进入 rollout。

所有数字与 scope flags 的 machine-readable authority 为
`simulation/mujoco/config/phase45_contact_consistent_rolling_v1.json`。formal/fresh replay append-only。

## REWORK — Equilibrium Compatibility Audit

DG45-EQ 失败后的追加审计见 [REWORK.md](REWORK.md)。它不增加 task、不运行 trajectory，
只用 tick0 的 `desired -> QP -> MuJoCo`、right xi 静态分解和 4D compatible-wrench
counterfactual 判断根因。审计配置为
`simulation/mujoco/config/phase45_equilibrium_compatibility_v1.json`；原任务表和停止顺序不变。

## Compatible-H0 Continuation

用户授权继续留在 Phase45，将 formal-v2 compatible wrench 作为新的 frozen equilibrium reference，
其余结构和参数不变。执行与结论见 [CONTINUATION.md](CONTINUATION.md)，machine-readable authority
为 `simulation/mujoco/config/phase45_contact_consistent_rolling_v2.json`。DG45-EQ 已 PASS，但
DG45-AUTH common projected authority 在全部 scale 上 QP/MuJoCo 反号，故 mandatory stop；
REAL/SHORT/10 s/REAUDIT 未进入。
