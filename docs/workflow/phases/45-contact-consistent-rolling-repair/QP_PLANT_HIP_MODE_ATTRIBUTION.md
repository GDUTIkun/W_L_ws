# Phase45 REWORK: QP-to-Plant Hip-Mode Attribution

状态：`review / attribution complete`  
日期：2026-08-30

## Scope

只使用 compatible-H0、Model B、Phase42 tick0 native snapshot 的现有
`slip_common_only` directional probe。`+/-` branch 和 `1/0.5/0.25` scale、equilibrium
wrench、unified task、gain、weight、plant、contact、friction 和 solver 均冻结。没有
step、trajectory、REAL/SHORT/10 s/Phase46、新 task/candidate/repair 或参数调整。

QP prediction 与 MuJoCo actual 使用相同的 fixed-state `J_xi`，分别将
`J_xi * Delta qacc_qp` 和 `J_xi * Delta qacc_mj` 按 leg DOF 与左右 common/differential
mode 分解。因此两侧的 hip/knee contribution 可直接比较。

## Result

| stage | hip-common contribution | knee-common contribution | common-leg residual |
| --- | ---: | ---: | ---: |
| QP | -0.0840223023 | +0.0834701198 | -0.0005521825 |
| MuJoCo | -4.1083402117 | -0.1867006407 | -4.2950408524 |

QP 的 hip 与 knee 同为 material（使用既有 `0.05` authority magnitude），但反号抵消；其
residual/hip ratio 是 `0.00657186`。MuJoCo realization 后两项均为负，residual/hip ratio 变为
`1.04544430`。所以并非 plant 从零创造了 hip mode；它放大 hip mode，并把 QP 的 hip--knee
cancellation 破坏为同号叠加。

完整 output 亦吻合：`G_QP[ddxi_c, slip_c]=-0.0003029496`，而
`G_MJ[ddxi_c, slip_c]=-4.2950932`。QP 与 MuJoCo 的 DOF/mode closure 分别为
`2.60e-18 / 2.08e-17` 与 `0 / 0`；branch、三档 scale、whole-dynamics/contact closure 和
reduction invariance 全部通过，fresh replay semantic error 为 0。

## Classification

`B-QP_CANCELLATION_BROKEN_IN_PLANT`。

这不授权修复，不改变 DG45-AUTH=FAIL 或 Phase45=`REWORK`，也不进入任何 trajectory gate。
正式 evidence：`evidence/automated/rework-qp-plant-leg-mode-attribution-formal-v4/`；fresh replay：
`rework-qp-plant-leg-mode-attribution-replay-v4/`。
