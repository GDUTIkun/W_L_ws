# Phase 46: Hip-Common-Safe Rolling Realization Repair — PLAN

状态：`review`  
日期：2026-08-31

## Goal

只验证一个 repair hypothesis：在 Phase45 compatible-H0 上，仅从 slip task 的 realization row
静态删除 bilateral hip-common direction，判断它是否足以关闭 actual plant 的 harmful
`slip-common -> ddxi-common` cross-coupling，同时保留有效 rolling/slip authority。

## Frozen Decisions

- 继承 Phase45 Model B、compatible equilibrium wrench、unified rolling semantics、state、contact、
  friction、torque limits、solver、gain、weight 与 10 ms/2 ms timing。
- 12D reduced acceleration 顺序为 base linear `0:2`、base angular `3:5`、left
  hip/knee/wheel `6:8`、right hip/knee/wheel `9:11`。
- 唯一 projection 为
  `h=(e6+e9)/sqrt(2)`、`P_safe=I-h*h^T`、`J_slip_new=J_slip*P_safe`。
  等价地只从 slip row 的 columns 6/9 同时减去 `(row[6]+row[9])/2`。它保留 hip
  differential 与其余所有方向，且不投影 slip observable、bias 或 target。
- xi task、contact model、interaction wrench、task scale、所有 penalty/weight 均不改变。
- 禁止第二 candidate、gain/weight/wrench tuning、新 task、planner/tracking/12D NMPC、
  coupled xi-slip redesign、precompensation、state-dependent/dynamic projection。

Phase45 frozen root cause 为 `G_MJ[ddxi_c,slip_c]=-4.2950931926`，其中 actual
hip-common contribution `-4.1083402117`（95.65%）；classification 为
`B-QP_CANCELLATION_BROKEN_IN_PLANT`。

## Gate Contract

严格执行 `EQ -> AUTH -> REAL -> SHORT -> 10 s -> post-repair authority reaudit`；任一 mandatory
gate FAIL 后不进入后续 gate。fixed-state AUTH 使用 xi-only/slip-only、common/differential、正负 branch
与 scales `1/0.5/0.25`，delta 仍为 `0.01 m/s2`。

- EQ：Phase45 阈值不变：`abs(ddxi_L/R)<=0.05 m/s2`、
  `abs(material tangent acceleration)<=0.01 m/s2`，并要求 bilateral contact/active、hard
  `<=1e-7`、slack `<=0.05`、torque margin `>=-1e-10 Nm`、whole dynamics/contact closure
  `<=1e-8`。
- AUTH：common actual cross `abs(G_MJ[ddxi_c,slip_c])<=0.1` 且相对 Phase45 至少降低
  90%；common actual slip self gain 必须为正并保留至少 Phase45 `0.0308422887` 的 50%；
  common actual xi self gain必须为正且 `>=0.05`。QP/MuJoCo common unified projected authority
  必须同号且 actual magnitude `>=0.05`；所有 branch/scale convergence relative `<=0.05`。
  同时完整报告 QP/MuJoCo 2x2 common transfer matrix、differential authority 与全部 cross gains。
- REAL：对 slip-common-only 的 QP 与 actual 使用 Phase45 相同 base、leg/non-wheel、native wheel、
  `(Jdot,v)`、逐 leg DOF/common-differential mode decomposition；闭合 `<=1e-10`。报告 actuator、
  contact、remaining/lhs generalized force 与 hip/knee common projection；whole dynamics/contact
  closure保持 `<=1e-8`。不得出现 knee-common/base/native-wheel/differential-leg 的 material migration，
  定义为任一替代 mode 对 `ddxi_c` 的绝对贡献 `>0.1`；native wheel qacc gain不得超过 Phase45
  absolute baseline，且 slip self不得低于 AUTH 门。
- SHORT/10 s：沿用 Phase45 223/1000 tick 与全部 contact、xi、wheel-rate、slip、base/full-body、
  WBC、wrench realization 阈值，不放宽。
- reaudit：仅在10 s全部 mandatory gate PASS 后，于 ticks `0/50/500/999` 对相同 fixed-state
  authority 和 regime signature 复审；判据与 AUTH 相同。
- fresh replay：对实际进入的 gates 做 append-only fresh replay，semantic max error `<=1e-11`；
  replay不授权进入已经被 mandatory FAIL 阻止的后续 gate。

## Tasks

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-T01 | 冻结 projection 与 Phase 边界 | PLAN、ROADMAP、source/CBM grounding | done |
| P46-T02 | 实现唯一 Phase46 profile | static rank-one slip-row projection；QP diagnostic 同 row；xi/contact/weights unchanged | done |
| P46-T03 | component verification | targeted build/test；projection exactness、idempotence、only-columns-6/9、QP algebra | done |
| P46-T04 / DG46-EQ | compatible-H0 equilibrium | tick0 closure完成；frozen EQ limits FAIL | done |
| P46-T05 / DG46-AUTH | full directional transfer audit | DG46-EQ FAIL，mandatory stop，未进入 | blocked |
| P46-T06 / DG46-REAL | physical realization audit | DG46-EQ FAIL，mandatory stop，未进入 | blocked |
| P46-T07 / DG46-SHORT | 223-tick rollout | DG46-EQ FAIL，mandatory stop，未进入 | blocked |
| P46-T08 / DG46-ROLL | 1000-tick rollout | DG46-EQ FAIL，mandatory stop，未进入 | blocked |
| P46-T09 / DG46-REAUDIT | post-repair authority | DG46-EQ FAIL，mandatory stop，未进入 | blocked |
| P46-T10 | formal/replay/review | dependency probe、py_compile、fresh replay、classification、REVIEW；PASS only creates RECORD | done |

### REWORK tasks — frozen nominal, limited increment

| ID | Task | Deliverable / validation | Status |
| --- | --- | --- | --- |
| P46-R01 | 实现唯一 incremental restriction | zero delta 维持原 rolling QP；仅 external slip-common delta 启用 frozen nominal hip-common hard equality | done |
| P46-R02 / DG46I-EQ | 重新验证 compatible-H0 equilibrium | 原 ddxi/tangent/contact/load/hard/slack/torque/dynamics/contact gates 全部 PASS | done |
| P46-R03 / DG46I-AUTH | frozen directional incremental audit | `+/-`、`1/0.5/0.25`、QP/MuJoCo decomposition 与 closure 完成；actual cross 未下降，FAIL | done |
| P46-R04 | formal、fresh replay 与 REVIEW | dependency probe、py_compile、build/test、non-finite audit、replay 完成；保持 REWORK | done |

## Classification

最终只能选择：

- `P46-A — static hip-common projection sufficient`
- `P46-B — cross-coupling reduced but insufficient`
- `P46-C — harmful mode migrated to another DOF/mode`
- `P46-D — slip authority destroyed by projection`
- `P46-E — multiple remaining mechanisms`
- `P46-U — evidence unreliable`

若 mandatory gate FAIL，保持 Phase46 `review/REWORK`，不创建 RECORD，也不自动进入 soft penalty、
coupled task、precompensation 或 dynamic projection。

## REWORK — Frozen Nominal, Limited Increment

本次 REWORK 不新开 Phase。Phase46 static row projection 已可靠地在 EQ FAIL，故唯一允许的替代是：
zero slip perturbation 继续走未投影的 Phase45 rolling QP；仅在 frozen tick0 `slip-common` external
task delta 非零时，追加一个无权重的 hard equality
`0.5*(nudot_left_hip+nudot_right_hip)= -0.009961062735978504 rad/s2`。
该右端是已冻结 compatible-H0 nominal QP hip-common 值，不是 xi/slip target offset、gain/weight
调整或 cross-gain precompensation；它只令 requested slip perturbation 的 QP hip-common increment 为零。
不运行 REAL/SHORT/10 s。本轮 gate 仅为 `EQ -> Incremental AUTH -> fresh replay`。
