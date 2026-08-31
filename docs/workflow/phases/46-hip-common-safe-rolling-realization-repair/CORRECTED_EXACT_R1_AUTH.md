# Corrected Production-Reference Exact-R1 — Fixed-State AUTH

日期：2026-08-31  
结论：`B-HARMFUL-CROSS-REMAINS` / `AUTH FAIL`

本审计仅使用 compatible-H0、tick0、frozen Model B 与 corrected `Pg_prod` candidate。controller、
projector、task、gain/weight、friction、solver、contact parameters 和 torque limits 均未修改；没有
运行 REAL、SHORT、10 s、trajectory、NMPC、R2 或新 repair。

## Directional definition

xi/slip 的 common/differential 四个独立输入均运行 positive/negative branches 与
`1.0/0.5/0.25` scales，原 delta 为 `0.01 m/s2`。每个 gain 严格定义为：

```text
gain = (probe_output - baseline_output) / signed_input_delta
```

baseline QP per-side output 为
`[-5.05757e-6,-1.00691e-5,-7.43890e-5,-7.69126e-5]`；baseline actual 为
`[-0.01933909,-0.04911103,0.00077029,0.00216294]`。完整原值、差分与 signed delta 保存在 JSON。

## Common transfer

rows 为 `[ddxi, slip_acceleration]`，columns 为 `[xi, slip]`：

```text
G_QP_common = [[ 0.9999404419, -0.0005568723],
               [-0.0005568723,  0.9947780694]]

G_MJ_common = [[ 0.9873663720, -0.1180399992],
               [-0.0149161034, -0.1401691458]]
```

actual harmful cross 相对 Phase45 降低 `97.2517477%`，但其绝对值 `0.118040 > 0.1`。
actual xi self `+0.987366` PASS；actual slip self `-0.140169` 反号 FAIL，且与 QP slip self
`+0.994778` 符号不一致。

## Differential transfer and contamination

```text
G_QP_differential = [[0.9999796889, 0.0000259263],
                     [0.0000259263, 0.9900199015]]

G_MJ_differential = [[ 0.9381805015, -0.1126091759],
                     [-0.0058599524, -0.0027965976]]
```

differential slip self 反号；common-slip 输入产生 differential
`[ddxi,slip]=[+0.1735731,-0.0899526]`，为 material common/differential contamination。
因为 common harmful cross 自身仍超 mandatory absolute gate，主分类为 B；self-authority loss 与
mode contamination 作为并存 finding 保存，不另行实施修复。

## Trust and closure

- branch split maximum：`1.18575e-11`，PASS；
- scale convergence maximum：`2.77212e-11`，PASS；
- 每个 probe R1 projector/range/reconstruction/operator semantics PASS，maximum residual
  `2.74016e-14`；
- 每侧均保持两个 3D contacts，normal-frame delta `0`，minimum friction margin
  `15.2007 N`；active constraints、rolling flags、model/controller/solver statuses 不变；
- fresh replay semantic maximum error：`0`。

## Decision

`AUTH FAIL / B-HARMFUL-CROSS-REMAINS`。R1 仍 exact closed，state/contact regime stable，证据可信。
下一允许动作仅为 `post-corrected-R1 authority attribution`；本轮严格停止。

Machine-readable evidence：
[formal](evidence/automated/corrected-exact-r1-auth-formal-v1/corrected-exact-r1-auth.json) 与
[replay](evidence/automated/corrected-exact-r1-auth-replay-v1/summary.json)。
