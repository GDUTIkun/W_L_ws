# Complete Phase 32 closure revalidation

DG39-02 有效，分类：`P39-D_x16_nonclosure_structurally_persists`。

| Family | max symmetric physical `ddxi` | Gate | Result |
| --- | ---: | ---: | --- |
| C1 leg configuration | `0.0848022 m/s²` | `0.05` | valid FAIL |
| C2 leg velocity | `2.07846 m/s²` | `0.05` | valid FAIL |
| C3 wheel spin rate | `1.65810 m/s²` | `0.05` | valid FAIL |
| wheel absolute angle | `6.04464e-5 m/s²` | `0.05` | valid PASS |

C1/C2/C3 相对 Phase 32 authority 的 ratio 分别为 `0.78283/0.98944/0.98970`；wheel-angle
ratio 为 `4.556e-5`。angle fixed-baseline-torque 最大差仅 `7.928e-9 m/s²`。所有 family 的
projection、full-body oracle、finite、bilateral-contact 及适用的 full/half validity gate 均成立；
angle runner 的 exit `2` 按冻结的旧 discrete-failure 语义正确接收，并由 Phase 39 数值 gate
重新裁决。

requested wrench parity 为 exact；最大 realized-wrench relative difference 为 `2.670e-6`，通过
`2%` gate。因此 requested-wrench 命题成立，也允许追加 physical-interaction-wrench 表述；
fixed-torque angle branch 独立确认 absolute angle effect 已消失。

结论只说明已知 collision/COM artifacts 移除后，x16 对 configuration、velocity、wheel-rate 三族
仍不 Markov closed；它不证明任何 12D controller 的 tracking 或 robustness PASS。
