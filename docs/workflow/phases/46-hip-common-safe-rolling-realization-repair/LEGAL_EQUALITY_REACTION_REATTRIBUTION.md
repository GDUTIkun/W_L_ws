# Legal Equality-Reaction Recovery + Post-Corrected-R1 Re-Decomposition

本轮只修复 diagnostic bookkeeping，不修改 production QP、torque 或 physical wrench。历史
post-hoc “QP equality reaction” 及由它得到的 equality gap 现标记为 **SUPERSEDED**；production
reduced QP 仍有效，corrected-R1 仍 CLOSED。

production operator 为 rank-4 `J_eq`，MuJoCo raw equality force space 为 rank 6。四个 production
mode 与 MuJoCo 最近四个 mode 的 principal angles 为 `0.000363–0.002201 rad`；按冻结的
`0.01 rad` operational common-mode tolerance，common dimension 为 4、production-only 为 0、
MJ-only 为 2。严格浮点代数交集同时保留为 0。

所有 baseline + 32 probes 的 `Q_eq=P_range(J_eq^T) r_full` range residual norm
`<=6.97e-17`，min-norm reconstruction residual norm `<=1.08e-16`。baseline projected dynamics
residual `4.48e-9`，raw orthogonal fraction `3.44e-8`，virtual-work residual `1.06e-16`。

slip-common 的旧 QP equality contribution `+0.307960415` 被合法值 `-0.067334201` 取代；MJ
common contribution为 `-0.063724030`，new common gap仅 `+0.003610170`，相对旧 gap
`-0.371684465` 移除约 `99.03%`。MJ-only equality contribution仅 `-1.91e-8`。

contact slip-common gap由旧 `-0.763262750` 更新为 `-0.749895431`，方向与量级稳健；但它只占
total discrepancy norm 的 `0.678965`。FREE slip-common self response另有约 `-0.388662` 的
QP/MJ gap，因此 contact不是唯一 material remaining mismatch，classification为
`E-MIXED-REMAINING-MECHANISMS`。

source closure `0`，branch split `5.83e-11`，scale convergence `1.33e-10`，fresh replay max
error `0`。R2 与 explicit-lambda controller repair均不授权。

Evidence：
[formal-v4](evidence/automated/legal-equality-reaction-reattribution-formal-v4/legal-equality-reaction-reattribution.json)、
[fresh replay-v1](evidence/automated/legal-equality-reaction-reattribution-replay-v1/summary.json)。
