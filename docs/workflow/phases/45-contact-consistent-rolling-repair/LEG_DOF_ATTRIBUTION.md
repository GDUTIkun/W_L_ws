# Phase45 REWORK: Compatible-H0 Slip-to-Xi Leg-DOF Attribution

状态：`review / attribution complete`  
日期：2026-08-30

## Scope

本 addendum 只展开已冻结的 compatible-H0、tick0 fixed-state `slip_common_only`
directional probe。Model B、native state、compatible equilibrium wrench、unified rolling
task、gain、weight、plant、6D contact、friction 和 solver 均未改变；沿用既有 `+/-`、
`1/0.5/0.25` probe，未 step、未运行 REAL/SHORT/10 s/Phase46，也未新增 candidate、task 或 repair。

对每个非 base、非 wheel velocity coordinate `i`，报告
`0.5 * sum_side(delta(J_xi[side,i] * qacc[i])) / delta_slip_common`。所有 coordinate
项之和严格回到既有 actual cross gain。

## DOF attribution

`G_MJ[ddxi_common, slip_common] = -4.295093192622`：

| leg/non-wheel DOF | contribution | share of cross gain |
| --- | ---: | ---: |
| right hip | -2.734463232556 | 63.6648% |
| left hip | -1.373567364992 | 31.9799% |
| right knee | -0.134758623924 | 3.1375% |
| left knee | -0.052303971151 | 1.2178% |
| left/right connect1 | 0 | 0% |
| left/right connect2 | 0 | 0% |

DOF sum 为 `-4.295093192622`，相对原 cross gain 的 closure 为
`8.88e-16`。

将左右同名坐标投影为 common/differential acceleration mode 后：hip common contribution 为
`-4.108340211684`（95.65%），knee common 为 `-0.186700640711`（4.35%）；hip/knee
differential 项分别只有 `+3.096e-4`、`-3.620e-4`，近乎相消，mode closure 为零。

## Attribution

这支持 **specific harmful leg mode**，更精确地说是一个以左右 hip 的 common leg mode
为主、并带小量 knee-common 分量的 mode；不支持“多个 leg/non-wheel DOF 共同形成的 distributed
full-body coupling”。右 hip 是最大的单一 DOF，但单独只占 63.66%，故完整物理解读是 bilateral
hip-common mode，而不是单侧 right-hip mode。

原 `base`、native wheel 和 `Jdot,v` 项仍分别为零/浮点残差；此处只把已确认的
`leg_nonwheel=-4.295093...` 项展开，未把 contact force 大小误作该 mode 的因果归因。

## Verification

正负 branch、`1/0.5/0.25` scale convergence、existing whole-dynamics/contact closure
和 reduction invariance 均保持通过。正式输出为
`evidence/automated/rework-leg-dof-attribution-formal-v2/`；fresh replay 为
`rework-leg-dof-attribution-replay-v2/`，semantic replay max error 为 0。

这不改变 DG45-AUTH 的 FAIL 或 Phase45 的 `REWORK` 状态，也不授权修复或后续 trajectory gate。
