# Closure-Conditioned Effective-Inertia / Precontact Response Attribution

## Decision

`D-MIXED-EFFECTIVE-INERTIA-AND-CLOSURE`.

上一轮 `C1-RAW-MASS-INERTIA-RESPONSE-MISMATCH` 继续有效，但其 scope 严格限定为
`RAW-TREE FIRST-MISMATCH ONLY`。将 production 与 MuJoCo 限制到同一个合法 rank-4 closure
tangent space 后，slip-common gap 为 `-0.3883828695`，仍保留 raw target
`-0.3886619350` 的 `99.9281984%`。因此 raw gap 没有被 common closure constraint 消除，存在
独立且 material 的 common-tangent effective-inertia mismatch。

在同一 MuJoCo mass operator 上从 common rank-4 切换到 native rank-6 后，slip-common 额外变化
`-0.04428122835`，为 raw target 的 `11.3932506%`，同样 material。它只表示 MuJoCo-only 两个
kinematic closure modes 的独立作用，不是旧 equality reaction mismatch。

本轮只做 attribution。没有修改 mass、COM、inertia、armature、model、closure、contact、controller、
QP、torque 或 compensation；`R2 AUTHORIZED = NO`。

## Operator provenance and gates

- frozen state/order/units/sign 与 authoritative `Delta Q_smooth` 沿用 compatible-H0 tick0；force
  provenance max gap `1.78e-13`。
- `M_prod`、`M_MJ`、`J_prod4`、`J_MJ_common4`、`J_MJ_native6` 与三个 conditioned operator 均保存
  在 machine-readable evidence。
- common4 使用 verified production rank-4 closure row space 在共同 16-DoF ordering 中的正交基，
  没有任选四条 MuJoCo rows。production/common ranks `4/4`，principal angles `<=4.87e-16 rad`，
  projector difference `0`，mutual containment `6.97e-16`，tangent projector difference
  `8.32e-16`。
- MuJoCo native rank为 `6`；production-vs-native 四个主角度为
  `0.0003629/0.0003671/0.0012730/0.0022011 rad`。
- fixed-state directional increment 的 `Delta b_closure=0`；全部 conditioned acceleration closure
  residual `<=1.42e-14`。
- production KKT-conditioned vs matched reduced response spectral/max-abs gaps为
  `6.36e-11/3.01e-11`，PASS。

## Three response semantics

slip-common authoritative direction：

| Comparison | full qacc gap norm | ddxi-c | slip-c | ddxi-d | slip-d |
| --- | ---: | ---: | ---: | ---: | ---: |
| raw `M_MJ^-1 Q - M_prod^-1 Q` | 1.016570084 | -0.008064917 | -0.388661935 | -0.007772844 | -0.009645165 |
| common4 `K_MJ4 Q - K_prod4 Q` | 1.012755439 | -0.007912876 | -0.388382870 | -0.007525078 | -0.009255749 |
| MJ-only `K_MJ6 Q - K_MJ4 Q` | 3.709800806 | -0.000223902 | -0.044281228 | +0.416942476 | -0.064995704 |

common4 slip-differential self gap为 `-0.000132496`，而 xi-common self gap为
`-0.000867364`；相对 raw target 均 nonmaterial。因此当前 probe set显示 common-mode-specific
effective-inertia response。conditioned joint-gap modes为 hip common `+0.0266815`、knee common
`+0.00301621`、wheel common `-0.0203743`；不继承 raw-tree `92.884%` share。

## Effective operator, tangent mass, and kinetic energy

`K_MJ4-K_prod4` 的 relative Frobenius gap为 `0.00581710`，spectral gap为 `14.1886531`。
dominant input/output 都由 `base-z (+0.660775) + base-ry (-0.409723) + left-hip (+0.400365)` 主导，
与 base-Z / bilateral hip-common excitation及 rolling/slip-common observable 一致。

使用唯一 matched tangent basis `T in R^(16x12)` 后，`T^T M T` relative Frobenius gap为
`0.0970278065`，spectral gap为 `0.585684816`；condition numbers为
`15614.36/15807.95`。dominant Delta-K、hip-common-like、wheel-common-like 和四个 deterministic
random tangent directions的 kinetic-energy audit最大 relative gap为 `0.187622671`，故 parity FAIL。
这满足继续 inertial-source attribution 的语义门，但不授权任何 inertial-parameter modification。

## Trust and authorization

`+/-` branch split `5.52e-11`，scale convergence `2.23e-10`；全部 corrected-R1 与 regime gates
PASS。observable rolling map parity为 `0`。fresh replay semantic error为 `0`。

Primary D 下本轮不做 body-level replacement/counterfactual，source保持 `NOT ATTRIBUTED`。下一允许
动作仅为 additional inertial-source attribution；同时保留 closure-model attribution candidate，
不得进入 repair。contact response继续 material且不是 unique remaining mismatch；R2 不是下一次
re-authorization candidate。

Evidence: [formal-v2](evidence/automated/closure-conditioned-effective-inertia-formal-v2/closure-conditioned-effective-inertia-audit.json)
and [fresh replay-v1](evidence/automated/closure-conditioned-effective-inertia-replay-v1/summary.json).
