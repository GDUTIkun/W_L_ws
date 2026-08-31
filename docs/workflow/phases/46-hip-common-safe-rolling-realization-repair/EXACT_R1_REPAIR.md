# Exact R1 point-force-image repair

结论：exact implementation 与 `COMP` PASS，`EQ` FAIL；Phase46 保持 `REWORK`。

本轮唯一 candidate 为 `P_G=G_pG_p^dagger`。`G_p` 由 compatible-H0/tick0 frozen Model B
左右 actual two-point positions、production contact frame 与 production aggregate wrench reference
构造。上一轮已冻结的 actual contact-line normal offsets 为 left
`2.1526549679640877e-4 m`、right `1.520424481146268e-4 m`；它们是本 Phase 的 frozen evidence
inputs，不是通用 contact estimator，也不授权 trajectory。

## Implementation and COMP

同一 orthogonal projector 一致进入 dynamics、37-row wrench cone、interaction-wrench realization
及 controller physical output/diagnostics。旧 pure-`Ml` gate 被 supersede：exact image 允许与 `Fr`
配对的微小 `Ml`，COMP 改为直接验证 actual `G_p/P_G`。

左右 `G_p` 均 rank 5，nonzero condition number 约 `50`。production/actual projector parity 为
`1.11e-15 / 1.55e-15`；mutual containment spectral residual 均 `<9.2e-16`；最大 principal
angle `<1.10e-15 rad`；actual missing-direction annihilation `<4.89e-16`；controller physical
wrench 的 point-force reconstruction residual norm `<=1.60e-14`。projector symmetry、idempotence、
EOM/contact algebra closure PASS；targeted build 与 historical profile tests PASS。因此
`Range(decision)=Range(G_p)`，R1 在本 candidate 中 exactly closed。

## EQ mandatory stop

同一 Phase45 compatible-H0/tick0 state 下，QP `ddxi_L/R` 为
`[-5.76281e-6,-8.22472e-6] m/s2`，actual 为
`[+0.0379952,-0.0752634] m/s2`。right 超过 frozen `abs(ddxi)<=0.05`，故 `EQ FAIL`。

其余 EQ 指标可信：actual material tangent acceleration
`[+0.000962777,+0.00451025] m/s2`、每侧 two contacts、hard `4.485e-11`、slack
`0.00167889`、minimum torque margin `1.99595 Nm`、whole-dynamics/contact closure
`7.11e-15 / 0`。按 mandatory stop 未运行 AUTH、REAL、SHORT、10 s 或任何 trajectory。

before/after causal evidence 对 actual delta `ddxi=[+0.0379952,-0.0752634]` 的 closure
`<=4.85e-14`，但只标为 `EXACT_R1_EQ_FAIL_CAUSAL_EVIDENCE`。它不设计 R2、不授权第二 repair，
且 `R2-CONTACT_RESPONSE_MISMATCH_AFTER_R1` 保持 `not re-authorized`。

Authoritative evidence：
[COMP/EQ formal-v3](evidence/automated/exact-r1-equilibrium-formal-v3/equilibrium-decision.json)、
[fresh replay-v3](evidence/automated/exact-r1-equilibrium-replay-v3/summary.json)、
[causal formal-v1](evidence/automated/exact-r1-causal-formal-v1/exact-r1-before-after-causal.json) 与
[causal replay-v1](evidence/automated/exact-r1-causal-replay-v1/summary.json)。

`exact-r1-equilibrium-formal/replay-v1` 使用了错误的 nominal-reference geometry；v2 修正 audit
geometry 后暴露 controller 仍使用 nominal offset。两组均 superseded/non-authoritative；v3 才同时
使用 frozen actual geometry 并通过 exact COMP。
