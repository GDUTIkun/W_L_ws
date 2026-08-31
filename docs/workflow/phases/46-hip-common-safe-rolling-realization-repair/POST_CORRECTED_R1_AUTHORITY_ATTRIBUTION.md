# Post-corrected-R1 fixed-state authority attribution

## Decision

`E-MULTIPLE-REMAINING-MECHANISMS`。corrected production-reference R1 继续 exact closed，全部
compatible-H0/tick0 probe 的 state/contact regime 稳定，但 slip-common discrepancy 不能由单一
first-level gap 支配：contact-response gap 对四维 discrepancy 的
`alpha=0.687409, r_res=0.321378, r_n=0.691449`；other constraint/passive gap 为
`alpha=0.312591, r_res=0.691449, r_n=0.321378`。因此 contact response 是最大贡献者但不是满足
dominance 条件的唯一 first mismatch，R2 不授权。

## Trust and closure gates

所有导数均为 `(probe-baseline)/signed_delta`，保留正负 branch 与 scales
`1/0.5/0.25`。q/qdot、M/bias、reduction、四维 observable map 的 probe-to-baseline max delta
均为 `0`；QP rolling map 与 physical slip map max delta 为 `0`。projector/range、point-force、
full/reduced operator 与 virtual-work gates PASS；双侧各两个 3D contacts，frame、active set、
solver signature、friction/contact regime 不变。QP、plant 与 gap 三个 causal balance 的全 probe
最大 closure 为 `2.89e-15`。formal-v3 的 fresh replay semantic error 为 `0`。

## Slip-common causal balance

四维次序为 `[ddxi_c, slip_c, ddxi_d, slip_d]`：

| term | values |
| --- | --- |
| QP output | `[-0.000556872, +0.994778069, -0.000273372, +0.000087332]` |
| MuJoCo output | `[-0.118039999, -0.140169146, +0.173573106, -0.089952591]` |
| free driver | `[-2.724939246, -16.441412048, -0.828721301, -6.890905076]` |
| QP contact | `[+2.784678119, +17.128229702, +0.811470957, +6.935248889]` |
| actual contact | `[+2.663903870, +16.364966952, +0.992101112, +6.830434104]` |
| contact gap | `[-0.120774250, -0.763262750, +0.180630155, -0.104814785]` |
| other gap | `[+0.003291123, -0.371684465, -0.006783676, +0.014774863]` |

QP slip self `+0.994778` 来自 large negative free slip `-16.441412`、large positive predicted
contact `+17.128230` 和 `+0.307960` QP-other 的平衡。plant contact cancellation 少
`0.763263`，同时 plant-vs-QP other 再少 `0.371684`，合成 actual `-0.140169`。因此反号是
contact 与 other 两个 material gaps 的合成，不是 free driver 自身的 first mismatch。

正 branch 的 torque gain `[LH,LK,LW,RH,RK,RW]` 为
`[+0.096206,+0.010870,-0.087324,-0.213237,-0.231362,-0.198651]`。free slip 主要由 wheel
尤其 RW 与 bilateral hip/leg realization 驱动；actual contact reaction 与 free slip 相反，说明
solver 正常抵消 torque-induced free contact motion。QP 仅 equality rows active，hard
`4.48e-11`、slack `0.0017324`、minimum torque margin `1.99882 Nm`。

## Slip-differential and contamination

slip-differential 的 contact gap 为
`[+0.001543,+0.005162,-0.112152,-0.998601]`，other gap 为
`[+0.000304,+0.003511,-0.000483,+0.005785]`。contact alignment
`alpha=1.005667`、residual `0.006796`，故 differential slip self 丢失由 contact response gap
支配。

common-slip 到 differential `[ddxi,slip]` 的 free driver 已为 `[-0.828721,-6.890905]`，QP
contact 预测 `[+0.811471,+6.935249]` 基本抵消；actual contact为
`[+0.992101,+6.830434]`，其 gap `[+0.180630,-0.104815]` 产生主要 residual contamination，
other gap `[-0.006784,+0.014775]` 次要。因此 contamination source 为
`multiple, contact-response dominant after a pre-existing free driver`。

## Xi-common healthy control

xi-common QP/MJ self 为 `+0.999940/+0.987366`。其 discrepancy 很小且 contact gap
`[-0.012954,-0.081858,+0.019247,-0.011342]` 被 other gap
`[+0.000380,+0.067499,-0.000747,+0.001743]` 大幅抵消；不像 slip-common，两个 gaps 没有同向
破坏 self authority。这是 slip direction 多出的 material causal pattern。

## Point-force and solver interpretation

production-reference `Gp_prod` 下，slip-common positive branch 的 aggregate-changing point-force
gap norm 为 left/right `1.48333/0.687654`，nullspace redistribution norm 仅
`6.47e-14/6.14e-14`；`Gp_prod delta_f_null <=3.71e-16`，wrench-gap closure
`<=1.77e-13`。故 mismatch 是 aggregate-changing，不是纯 point-force redistribution。bilateral
normal-positive contact 保持，minimum friction margin `15.2011 N`。

`IS CONTACT-RESPONSE MODEL THE FIRST MISMATCH: NO`（它是最大单项，但 slip-common 尚有
31.3% aligned other gap）。`IS THERE EVIDENCE OF A SOLVER BUG: NO`。未进入 conditional KKT；
primary torque source 仅能定位到 actuator realization（RW 与 bilateral leg/hip dominated），不将
未运行的 task/KKT decomposition 猜成结论。

## Authority and stop

- `R2-CONTACT_RESPONSE_MISMATCH_AFTER_CORRECTED_EXACT_R1`: **NOT AUTHORIZED**。
- next repair layer: **not selected**；需要 additional fixed-state attribution 分离
  `other_constraint` 与 `passive/applied`，之后才能定义一个 Phase46 REWORK repair candidate。
- 未修改 controller、gain/weight、friction/solver/contact parameters；未运行 REAL、SHORT、10 s、
  trajectory 或 NMPC；未实施 repair。

Authoritative evidence: [formal-v3](evidence/automated/post-corrected-r1-authority-attribution-formal-v3/post-corrected-r1-authority-attribution.json)
and [fresh replay-v3](evidence/automated/post-corrected-r1-authority-attribution-replay-v3/summary.json).
