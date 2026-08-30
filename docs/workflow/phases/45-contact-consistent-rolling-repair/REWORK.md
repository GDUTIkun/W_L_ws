# Phase45 Rework: Equilibrium Compatibility Audit

状态：`review`  
日期：2026-08-30

## 唯一问题

本 REWORK 不建立 Phase46、不增加 controller task，只回答 tick0 的
`right ddxi=-0.0533965 m/s2` 来自 fixed interaction wrench equilibrium mismatch、
xi realization mismatch，还是两者都有。

## 冻结口径

- 仅使用 Phase42 authority 的 tick0 fixed state；不积分 trajectory。
- 保持 Phase45 unified task、gain、weight、solver、plant、contact、friction 与 initial state 不变。
- `xi realization` 分成两层：`desired -> QP realized` 是 task realization；
  `QP realized -> MuJoCo actual` 是 plant realization gap，不能直接当作独立 task failure。
- counterfactual 只复用 Phase43 已冻结的 4 个 sagittal wrench 分量：
  `left/right Fx`、`left/right Ty`，边界仍为 `+/-10 N`、`+/-1 Nm`。
  四个等式为两侧 `ddxi=0` 与两侧 material `a_t=0`；不使用正则化、candidate search、
  gain/weight tuning。
- compatible 解必须同时通过 `1e-7 m/s2` equality、WBC hard/slack/torque、
  whole dynamics/contact closure 与 `1e-7 m/s2` decomposition closure。

## 结果

fixed wrench 下：

| quantity | desired | QP realized | MuJoCo actual |
| --- | ---: | ---: | ---: |
| left ddxi [m/s2] | 0 | 9.479e-11 | -0.0103356 |
| right ddxi [m/s2] | 0 | 3.747e-11 | -0.0533965 |
| left material a_t [m/s2] | 0 | -1.894e-9 | -0.000557242 |
| right material a_t [m/s2] | 0 | -1.847e-9 | -0.00134511 |

right MuJoCo `ddxi` 静态分解为：base `-6.63e-19`、leg/non-wheel
`-0.05339650936`、wheel `0`、Jdotv `1.32e-16 m/s2`，闭合误差为0。
QP 同一分解的 right sum 为 `3.747e-11 m/s2`。因此右侧残差不是 wheel-column 或
Jdotv 项，而是 fixed wrench/torque 进入 actual constrained plant 后的 leg acceleration。

compatible counterfactual 相对 fixed request 的改变量为：

- left/right `Fx = -0.0631927 / -0.187486 N`；
- left/right `Ty = +0.00259355 / +0.00279667 Nm`。

重新求解相同 WBC 后，MuJoCo `[ddxi_L, ddxi_R, a_t_L, a_t_R]` 为
`[4.06e-14, 2.87e-14, 1.04e-16, -6.94e-17]`。hard violation
`4.68e-11`、maximum normalized slack `0.001344`、minimum torque margin
`1.99825 Nm`、whole dynamics residual `2.13e-14`、contact reconstruction residual `0`，
全部通过。right fixed residual 的消除比例为 `0.99999999999946`。

## 结论

分类为 `FIXED_WRENCH_EQUILIBRIUM_MISMATCH`。fixed case 的 QP task realization
误差只有 `3.75e-11 m/s2` 量级，不支持独立的 xi task realization mismatch。
fixed wrench 下确实存在 `QP -> MuJoCo` realization gap，但 compatible wrench 在不改 task、
gain 或 weight 的情况下把它完全关闭，因此该 gap 是 equilibrium incompatibility 的表现，
不是第二个并列根因。

正式 authority 为 `evidence/automated/equilibrium-compatibility-formal-v2/`，fresh replay 为
`equilibrium-compatibility-replay-v2/`；semantic max error 为0。v1 数值与结论相同，但汇总读取
temporary probe、归档 probe 为随后确定性重跑的副本，provenance 不够直接，故 append-only 保留为
rejected evidence，不参与 authority。

本 REWORK 只关闭根因审计。Phase45 原 repair 仍未通过 DG45-EQ/AUTH/rollout，故总体 REVIEW
保持 `REWORK`，不创建 RECORD，不进入 Phase46。

## Compatible-H0 continuation

后续按用户授权把本审计解冻结为 H0 reference。DG45-EQ PASS，但 DG45-AUTH 的 common projected
gain 为 `G_QP=+0.998203`、`G_MJ=-1.875899`，且三档 scale 收敛，因此在 AUTH mandatory stop。
详见 [CONTINUATION.md](CONTINUATION.md)。

## Compatible-H0 DG45-AUTH attribution addendum

针对 common `G_QP=+0.998203`、`G_MJ=-1.875899` 的 fixed-state split 已完成。xi-only self 为
`+0.999969 -> +0.508522`，slip-only self 为 `+0.997042 -> +0.0308423`，均未翻号；MuJoCo 的
`slip_common_only -> ddxi_common=-4.295093` cross term 使重新组合的 unified projection 反号。

广义力 contact share 约0.772、contact/actuator norm ratio约3.39，但 contact--actuator cosine 为
`-0.0250/-0.1764`，不满足冻结的 directional cancellation 门。因此分类为
`C-CROSS_COUPLING_REVERSAL`，不是 `D-CONTACT_REDIRECTION_DOMINANT`。详细输入、矩阵、closure 与
scope contract 见 [AUTHORITY_ATTRIBUTION.md](AUTHORITY_ATTRIBUTION.md)。这只解释 DG45-AUTH，既不
改变该 gate 的 FAIL，也不授权 REAL/trajectory/Phase46。
