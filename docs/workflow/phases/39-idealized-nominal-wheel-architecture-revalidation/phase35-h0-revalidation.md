# Phase 35 H0 revalidation

DG39-03 有效，分类：`P39-F_H0_spin_drift_persists`。

Model B 使用原 Phase 35 C++ loop、fixed equilibrium wrench、150-tick request 和 live `±1 rad`
gate。运行在 tick `96` 由 canonical index `5`（right wheel）首先拒绝；right/left wheel delta
最大变化分别为 `1.03679/0.319619 rad`，均超过 `0.1 rad` drift gate。

拒绝前 bilateral contact、finite、hard、slack 和 torque gates 全部通过：最大 hard violation
`4.312e-10`，最大 normalized slack `2.141e-4`，最小 torque margin `1.99998 Nm`。right wheel
在 tick `95` 进入 near-boundary，拒绝 signed margin 为 `-0.0367881 rad`。因此 workspace
rejection 之前没有 contact、WBC resource、torque/slack 或 base-state validity failure。

formal-v2/replay-v2 的 Phase 39 H0 summary 完全相等，两个 97-row CSV 在排除真实 wall-clock
`wbc_time_s` 后逐字段相等；runner 内部 A/B replay numeric error 为 `0`。
