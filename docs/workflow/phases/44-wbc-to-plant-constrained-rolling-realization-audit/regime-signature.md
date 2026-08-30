# Phase 44 Addendum — Frozen Regime Signature

## Equality fields

每个 probe 的 equality signature 是以下离散字段的 canonical JSON：

- `contact_topology`: 每侧 existence/count 与排序后的 `geom-pair/dim`；contact point position和原始
  MuJoCo order不参与 equality。
- `contact_load`: normal load 为 `nonpositive/positive`；friction utilization 为
  `unloaded/interior/near_limit`；penetration为 `separated/near_zero/penetrating`；tangential slip为
  `negative/stick_band/positive`。
- `qp_inequalities`: assembled inequality rows `12..103` 的 lower/upper `distance<=1e-7` code、
  torque/contact-friction/acceleration group count、六路 torque-bound bit；multiplier
  signature=`unavailable`。
- `solver_task`: model/controller/solver success与status、candidate/profile、enabled task rows、QP variable
  count `42`、constraint row count `104`、maximum normalized slack 的
  `inactive/nonmaterial/material` state。

## Frozen thresholds

| Quantity | Threshold |
| --- | --- |
| assembled inequality / torque bound active | `<=1e-7` distance to bound |
| positive normal load | `>1e-6 N` |
| friction near limit | utilization `>=0.95` |
| penetration near-zero band | `abs(distance)<=1e-8 m` |
| slip stick band | `abs(v_t)<=1e-5 m/s` |
| slack inactive / material | `<=1e-7` / `>=0.02` |
| directional convergence | relative gain difference `<=0.05` |
| smooth two-sided branch | relative `G+` vs `G-` difference `<=0.05` |

连续 load、force、penetration、slip、contact position、inequality margin 仍写入 evidence，不能单独触发
regime change。signature 比较不使用 solver multiplier；该 authority 在当前 solver 中不可用。
