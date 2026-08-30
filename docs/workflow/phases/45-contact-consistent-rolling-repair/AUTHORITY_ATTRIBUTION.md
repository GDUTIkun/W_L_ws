# Phase45 REWORK: Compatible-H0 Common-Channel Attribution

状态：`review / attribution complete`  
日期：2026-08-30

## 范围与冻结

这是 tick0 fixed-state 的归因 addendum，不是 repair、task 或 candidate。compatible H0 wrench、
Phase42 authority tick0 state、Model B、Phase45 unified rolling task、gain、weight、6D contact model、
plant、friction、torque limits 和 solver 全部冻结。每个 controller invocation 都从同一 native snapshot
恢复，且只产生 `pre_command/post_command`；没有 `stepped`，没有 REAL/SHORT/10 s/re-audit。

诊断输入仅把既有 unified common direction 拆开：`xi_common_only=[+0.01,+0.01,0,0]` 与
`slip_common_only=[0,0,+0.01,+0.01] m/s2`。各自使用 `+/-` 及 scale `1/0.5/0.25`；它们不构成
新 controller candidate。

## Common transfer

行是 `[ddxi_c, a_t,c]`，列是 `[xi_common_only, slip_common_only]`：

```text
G_QP = [[+0.99996895, -0.00030295],
        [-0.00030295, +0.99704222]]

G_MJ = [[+0.50852155, -4.29509319],
        [+0.00393063, +0.03084229]]
```

两侧 branch 的最大 directional split 为 `2.57e-11`（xi）和 `1.43e-12`（slip），三个 scale 的最大
convergence relative 为 `1.23e-10`，故可中心报告上述矩阵。

1. `xi-common only` self authority：`+0.99996895 -> +0.50852155`，未翻号。
2. `slip-common only` self authority：`+0.99704222 -> +0.03084229`，未翻号。
3. 两 self channel 均正常；但 unified direction 的原投影为矩阵全项平均，重建为
   `G_QP=+0.9982026331`、`G_MJ=-1.8758993629`。主导项是
   `G_MJ[ddxi_c, slip_common_only]=-4.29509319`，因此反号来自 cross-coupling。

## Contact generalized-force attribution

每个 branch/scale 都记录了 QP contact generalized force、MuJoCo contact generalized force、
actuator/contact/remaining generalized-force balance、native wheel qacc，以及 xi 的 base / leg-nonwheel /
wheel / Jdot-v 分解。reduced directional balance最大残差为 `1.43e-11`，whole-dynamics/contact closure
和 reduction invariance 的最大值为 `4.98e-14`，因此这些数值可用。

MuJoCo contact generalized-force norm 是 actuator norm 的约 `3.39` 倍，force share 约 `0.772`；但它
与 actuator generalized-force 的 cosine 仅 `-0.0250`（xi-only）和 `-0.1764`（slip-only），未达到
冻结的 cancellation/redirection 门槛 `<= -0.5`。所以 contact reaction 确实很大，却没有定量证据表明
它以与 actuator 相反的 generalized-force 方向主导了翻号。不得把 QP 正、MuJoCo 负仅归因为 contact
cancellation/redirection。

## Classification

`C-CROSS_COUPLING_REVERSAL`。

`A-XI_SELF_REVERSAL`、`B-SLIP_SELF_REVERSAL` 与
`D-CONTACT_REDIRECTION_DOMINANT` 均不成立；没有实施修复，也不改变 DG45-AUTH 的定义或其 FAIL 状态。

正式 evidence 是 `evidence/automated/rework-authority-attribution-formal-v1/`，fresh replay 是
`rework-authority-attribution-replay-v1/`，semantic max error 为0。
