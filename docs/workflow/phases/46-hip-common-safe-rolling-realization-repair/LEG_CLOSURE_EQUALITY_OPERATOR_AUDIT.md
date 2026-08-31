# Bilateral leg-closure equality-response operator audit

## Decision

`D-QP-CONSTRAINED-REDUCTION/REACTION-MISMATCH`。QP 与 MuJoCo 描述同一 bilateral closure
geometry、Jacobian row space 与 `Jdot*v`；MuJoCo absolute stabilization target 非零，但其 full
coupled linear influence 对 slip-common equality gap 仅为 `-0.00491981`（`1.32365%`），不是
material root cause。第一次 material mismatch 出现在 QP constrained-reduction reaction
reconstruction：slip-common QP reaction 的 `99.9233%` 位于
`Range(J_eq^T)` 之外，且与 full coupled rigid equality reaction 的 relative difference 为
`0.999967`。

本轮未修改 site/equality、reduction、controller 或 solver parameters；没有实施 repair。overall
`E-MULTIPLE-REMAINING-MECHANISMS` 与 R2-not-authorized 均保持。

## Geometry and row semantics

左右均为 3D `connect` equality：

- left: equality ID `1`，efc rows `0..2`，sites `left_connect2_site` / `left_calf_site`，body IDs
  `11/8`；relative position `[-8.13135e-5,0,-1.63486e-4] m`，relative velocity zero。
- right: equality ID `2`，efc rows `3..5`，sites `right_connect2_site` / `right_calf_site`，body IDs
  `6/3`；relative position `[-8.07009e-5,-2.78e-17,-1.61975e-4] m`，relative velocity zero。

QP reduction 与 MuJoCo equality 都直接使用上述 site-pair position Jacobian difference。raw max
与 spectral difference 均为 `0`；rank 均为 `6`，mutual containment `1.44e-15`，nullspace
projector difference `0`。principal angles 的数值上界 `3.33e-8 rad` 来自 `acos` conditioning，
不表示 row-space mismatch。row-normalized raw/spectral difference亦为 `0`；deterministic
virtual-work identity residual为 `1.73e-18`。

## Jdot*v and acceleration targets

冻结 qdot 下两侧 `Jdot*v` 均为 exact numerical zero，QP/MJ gap zero。QP rigid-closure target 为
`[0,0,0,0,0,0]`。MuJoCo `efc_aref` 为：

```text
[+0.203471640, 0, +0.409091868,
 +0.201925508, 6.94e-14, +0.405285378]
```

它由非零 position residual 与 equality `solref/solimp` spring-damper stabilization 产生；官方
MuJoCo semantics 是 `efc_aref = -b*(Jv)-k*r`，constraint cost 使用
`J*qacc-efc_aref`。full frozen equality+contact metric传播该 target gap 后，四维 influence 为
`[-0.00314486,-0.00491981,-0.00007505,+0.00006083]`，linear residual `1.67e-16`。其 slip_c
只占 authoritative `-0.371684465` 的 `1.32365%`，故 target mismatch 记录为 secondary
stabilization offset，不冻结 B。

MuJoCo field semantics source: [official computation documentation](https://mujoco.readthedocs.io/en/latest/computation/)
and [official API types](https://mujoco.readthedocs.io/en/latest/APIreference/APItypes.html).

## Coupled reaction comparison

diagnostic rigid solve保留 frozen `M/B`、全部 16 contact efc rows 与 6 equality rows，并使用
baseline-subtracted directional RHS `J*dqacc=0`。全 probe KKT residual `<=2.54e-14`。
slip-common 四维 equality contributions为：

| source | `[ddxi_c, slip_c, ddxi_d, slip_d]` |
| --- | --- |
| QP reconstructed | `[-0.060296,+0.307960,+0.016977,-0.044256]` |
| MuJoCo row-wise | `[-0.057005,-0.063724,+0.010193,-0.029482]` |
| coupled rigid | `[-0.059654,-0.064461,-0.001791,-0.018213]` |
| MJ-QP gap | `[+0.003291,-0.371684,-0.006784,+0.014775]` |

QP-vs-rigid generalized-force relative difference `0.999967`；MJ-vs-rigid 为 `0.111808`。更早且
更直接的 gate 是 QP reconstructed force 对 equality row-force space 的 residual fraction
`0.999233`：它包含 material common-translation/base generalized forces，而 site-difference
equality 对共同平移不做功。因此当前 QP residual 不能作为 full rigid bilateral equality reaction。

这定位到 constrained reduction / reaction reconstruction layer；按 stop rule 不继续修改或重新
设计 reduction，也不把 MuJoCo 的次级 `11.18%` rigid difference升级成另一个 primary mechanism。

## Contrasts and closure

slip-differential equality gap仍小：
`[+0.000304,+0.003511,-0.000483,+0.005785]`；同一 QP range/reaction semantic issue存在，但对
目标 observable投影很小。xi-common equality gap
`[+0.000380,+0.067499,-0.000747,+0.001743]` 与 contact slip_c gap反向，继续解释 healthy
control中的抵消；slip-common equality/contact gaps则同向。

R1/regime PASS；reaction algebra closure exact，branch split `1.80e-10`，scale convergence
`4.16e-10`，fresh replay semantic error `0`。

## Authority and stop

- geometry root cause: NO。
- acceleration-target root cause: NO（secondary 1.32% influence）。
- equality response-law root cause: NO at the first-mismatch gate。
- QP constrained reduction/reaction model root cause: YES。
- solver bug evidence: NO。
- R2 authorized: NO。
- repair law candidate: QP constrained reduction/reaction model。
- next allowed action: define one Phase46 REWORK repair candidate。

Authoritative evidence: [formal-v4](evidence/automated/leg-closure-equality-operator-audit-formal-v4/leg-closure-equality-operator-audit.json)
and [fresh replay-v4](evidence/automated/leg-closure-equality-operator-audit-replay-v4/summary.json).
