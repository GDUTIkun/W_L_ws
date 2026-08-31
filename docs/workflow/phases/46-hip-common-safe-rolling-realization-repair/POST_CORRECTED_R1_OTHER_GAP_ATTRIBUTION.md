# Post-corrected-R1 other-gap attribution

## Decision

`OTHER-GAP CLASSIFICATION = D-NONCONTACT-CONSTRAINT-GAP`。上一轮 slip-common
`other gap=[+0.003291123,-0.371684465,-0.006783676,+0.014774863]` 全部由 bilateral
`left_leg_closure/right_leg_closure` equality-response 的 QP-vs-MuJoCo gap 重建；slip_c signed
contribution 为 `-0.371684465`，fraction `1.00000000000046`。它不是 passive、applied、external、
bias、numerical remainder 或 contact bookkeeping overlap。

这不 supersede 上一轮 overall `E-MULTIPLE-REMAINING-MECHANISMS`：contact response 与独立
non-contact equality response 仍是两个 material first-level mechanisms。R2 不是 re-authorization
candidate，且继续不授权。

## Dynamics semantics and trust gates

MuJoCo authoritative convention 从项目 Oracle 源码与直接 force balance确认：

```text
M*qacc + qfrc_bias
= qfrc_actuator + qfrc_passive + qfrc_applied + qfrc_constraint
```

`xfrc_applied` 在每次 evaluate 前显式清零。QP 侧用同一 frozen `M/bias/B/S`、production contact
operator；bilateral leg closure 由 plant-constrained reduction 隐式建模，其 generalized reaction
由 QP dynamics residual 唯一重建。MuJoCo 侧 primary decomposition 直接使用
`efc_J.T @ efc_force`，其中 equality rows 为 `efc_type=0`、IDs `1/2`，对应
`left_leg_closure/right_leg_closure`；contact 为 pyramidal rows `efc_type=6`。

q/qdot、M、bias、observable map、R1、operator/reference、contact/regime gates 全部保持 PASS。
全 probe constraint-row closure `<=1.42e-14`，contact rows 与 authoritative point-contact
generalized force closure `<=2.84e-12`，QP/MJ dynamics force closure `<=2.84e-12`，other-gap
observable closure `<=7.54e-14`。branch split `4.06e-10`、scale convergence `1.52e-9`；均通过
冻结 gates。fresh same-state replay semantic error 为 `0`。

## Slip-common four-output channel closure

四维顺序为 `[ddxi_c, slip_c, ddxi_d, slip_d]`：

| channel | QP | MuJoCo | MJ-QP gap |
| --- | --- | --- | --- |
| contact rows | `[+2.784678,+17.128230,+0.811471,+6.935249]` | `[+2.663904,+16.364967,+0.992101,+6.830434]` | `[-0.120774,-0.763263,+0.180630,-0.104815]` |
| leg-closure equality | `[-0.060296,+0.307960,+0.016977,-0.044256]` | `[-0.057005,-0.063724,+0.010193,-0.029482]` | `[+0.003291,-0.371684,-0.006784,+0.014775]` |
| passive | zero | zero | zero |
| applied | zero | zero | zero |
| external/body-applied | zero | zero | zero |
| bias delta | zero | zero | zero |
| limit/friction-loss/other constraint | zero | zero | zero |
| numerical remainder | — | `<=5.88e-13` | `<=5.88e-13` |

最高信息增益分流因此为 `CONSTRAINT`：smooth-minus-actuator 仅 numerical zero。previous other gap
不是 plant contribution 本身；QP 已通过 reduction 建模同一 bilateral closure physics，material
量是 equality reaction 的 `MJ-QP` gap。

## Independence and overlap

`IS OTHER GAP INDEPENDENT OF CONTACT = YES`。equality rows 与 contact rows 类型、IDs 与
generalized-force reconstruction彼此分离；contact-point reconstruction不能代数重建 equality
channel；row-wise total、point-contact parity、same-state torque replay、branches/scales 均闭合。

`IS THERE CONTACT BOOKKEEPING OVERLAP = NO`。旧 secondary
`qfrc_constraint - point-contact` subtraction 与本次 row-wise result 一致，但没有用作 primary
证据。

## Contrasts

slip-differential 仍为 contact dominated：contact gap 的 slip_d `-0.998601325`，equality/other
gap仅 `+0.005784825`。xi-common 中 contact slip_c gap `-0.081857848` 与 equality gap
`+0.067498617` 反向抵消；slip-common 中两者为 `-0.763262750/-0.371684465`，同向破坏 slip
authority。这解释了 healthy xi 与 failed slip direction 的差异。

## Stop and authority

- primary concrete source: bilateral leg-closure equality response model gap。
- overall classification remains `E-MULTIPLE-REMAINING-MECHANISMS`。
- contact response is not the unique first mismatch。
- `R2 CANDIDATE FOR RE-AUTHORIZATION = NO`; `R2 AUTHORIZED = NO`。
- next repair layer: non-contact equality-response model gap；本轮不实施 repair。
- next allowed action: define one Phase46 REWORK repair candidate。

Authoritative evidence: [formal-v2](evidence/automated/post-corrected-r1-other-gap-attribution-formal-v2/post-corrected-r1-other-gap-attribution.json)
and [fresh replay-v2](evidence/automated/post-corrected-r1-other-gap-attribution-replay-v2/summary.json).
