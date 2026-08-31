# Common-Tangent Inertial / Kinematic-Assembly Source Attribution

## Decision

`H-KINEMATIC-INERTIA-ASSEMBLY-MISMATCH`.

四组合 factorial 在固定 common rank-4 closure、matched `T`、`Delta Q_smooth` 与 observable map
下重现 authoritative gap：`G_MM-G_PP=-0.3883828695107319`。family main effects为：

- inertial parameters: `-0.008617569633`，signed fraction `+2.2188336%`，secondary/nonmaterial；
- kinematic/tree assembly: `-0.379774794691`，signed fraction `+97.7836111%`，material；
- interaction: `+0.000009494813`，signed fraction `-0.0024447%`，nonmaterial。

因此不进入 inertial parameter group/body attribution，也不授权 inertial repair。

## Provenance gates

production 与 MuJoCo inventory 均为同一 11-body tree，name/ID/parent逐项匹配。production `kBody`
与 MuJoCo compiled runtime fields的 mass、principal inertia逐项一致；armature全部为零。已在各自
body Jacobian与source frame/reference上独立重建：

- `M_prod_rebuilt` vs runtime max abs `<=2.22e-16`；
- `M_MJ_rebuilt` vs runtime max abs `<=1.11e-16`。

总质量均为 `6.4344 kg`。whole-body COM 的 MJ-minus-production gap为
`[-7.04691e-6, 0, -1.21015e-6] m`，主要来自 centered-wheel COM source revision，但其整个
inertial family只贡献 target的 `2.219%`。

## Dominant assembly source

dominant source是 floating-base generalized translational velocity的reference semantic：production
使用 `base_control_frame` point velocity，MuJoCo free joint使用 `base_body` origin velocity。两点在
frozen state的world offset为真实 control-frame offset；source-level Jacobian transport为
`v_body = v_control + skew(r_control) omega`，作用于所有下游body Jacobians，不是 generalized-M
block splice。

把 MuJoCo body-origin Jacobians transport到 production control-point reference后：

- explained slip-c amount `-0.379765413752`，signed fraction `97.7811957%`；
- remaining common4 gap `-0.008617455759`（`2.219%`）；
- tangent mass relative gap从 `0.0970278065` 降至 `1.53638e-5`；
- effective-operator relative gap降至 `5.74414e-4`；
- dominant Delta-K input/output alignment均为 `0.999809624`；
- dominant tangent kinetic-energy gap从 `-0.154433038` 降至 `-1.01665e-6`。

这同时满足 response、kinetic-energy 与 Delta-K mode验证。remaining gap与 factorial inertial
secondary effect一致，不需要 mixed-source closure。

## Authorization

source已定位，允许下一轮定义一个 source-specific diagnostic/repair candidate，但本轮没有修改
kinematic model、production source、MuJoCo XML、controller、QP、contact或closure。
`INERTIAL-PARAMETER MODIFICATION AUTHORIZED=NO`，`KINEMATIC-MODEL MODIFICATION AUTHORIZED=NO`，
`R2 AUTHORIZED=NO`。MJ-only rank-2 closure与contact material结论保持独立冻结。

Evidence: [formal-v3](evidence/automated/common-tangent-source-attribution-formal-v3/common-tangent-inertial-kinematic-source-attribution.json)
and [fresh replay-v1](evidence/automated/common-tangent-source-attribution-replay-v1/summary.json).
