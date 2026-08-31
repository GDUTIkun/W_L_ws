# Repair direction

`RECOMMENDED REPAIR LAYER`：**actual point-contact-realizable force/wrench parameterization**。

下一 repair 应首先把 WBC contact decision 限制在 actual two-point force image，例如直接优化
point forces，或使用显式 realizable-wrench allocation/subspace parameterization。这里只选择 layer，
不冻结具体 repair design，也不修改 controller、plant 或 contact parameters。

不要先做：

- hip-common projection、hard suppression 或 soft penalty；
- hip task redesign；
- inverse `R_c` / precompensation；
- friction、solref/solimp、solver 或 contact-model tuning。

理由：这些动作都位于已证明的 first mismatch 之后，或已被 Phase46 既有 evidence 排除为主要方向。

## Executed candidate result

已实现独立 `kPhase46PointRealizableRolling` candidate，以
`P_w=diag(I3,I3-a*a^T)` 将 contact wrench 严格限制到 frozen two-point force image，并一致用于
dynamics、wrench cone、interaction-wrench task 与 controller physical solution。组件层 rank-5、
projector algebra 与 axial moment closure PASS，但 `DG46P-EQ` FAIL：actual left/right ddxi 为
`+0.0377309 / -0.0753842 m/s2`，右侧超过 `0.05` 门。

因此“先修 realizability layer”的方向已执行，但该最小 subspace candidate **未获批准**。不得用
weight/gain/solver/contact tuning 推过 EQ。AUTH/REAL/rollout 均未授权；下一 candidate 需要先对
post-R1 equilibrium response mismatch 建立新的 attribution/decision gate，不能从本次 FAIL 自动猜定。

该 attribution 已完成并冻结为 `R2-CONTACT_RESPONSE_MISMATCH_AFTER_R1`：before/after state、mass、
xi map 均逐值不变，actual ddxi delta 的 causal balance closure `3.11e-14`，contact-response gap
norm 为 observed delta 的 `2.9883` 倍。此结论只冻结下一问题层，不授权 inverse response、经验
precompensation 或 task/solver tuning；下一 candidate 仍需单独定义可验证的 response model 与
authority-preservation contract。
