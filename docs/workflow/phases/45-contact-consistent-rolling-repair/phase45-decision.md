# Phase45 decision

结论：`P45 structure FAIL at DG45-EQ`。

单一 unified task 的actual-contact oracle可信且baseline provenance通过，但zero-error nominal state的
right wheel-center residual derivative超过formal前冻结门。冻结分类P45-A/B/C/E/U没有为“可信证据下、
rollout前mandatory EQ/AUTH gate直接失败”分配字母；因此不虚构映射到P45-U（U只用于证据不可信），
保留规范中的原文 `P45 structure FAIL`。不添加第二repair、不改gain/weight/wrench/plant/contact。

Phase45 REWORK 的 tick0 compatibility audit 已解释该分离：fixed case 的 QP xi row 实现为0，
但 fixed interaction wrench 与 actual constrained equilibrium 不相容，产生几乎纯 leg/non-wheel 的
right `ddxi=-0.0533965 m/s2`；4D compatible wrench counterfactual 在不改 task/gain/weight 时把两侧
actual `ddxi` 与 material `a_t` 同时关闭到 `4.06e-14 m/s2` 以内。故根因冻结为
`FIXED_WRENCH_EQUILIBRIUM_MISMATCH`，而不是并列的 xi task realization mismatch。

compatible wrench 随后作为 frozen H0 reference 进入 Phase45 continuation，DG45-EQ PASS；但
DG45-AUTH common projected gain 在 `1/0.5/0.25` scale 均出现可信的 QP/MuJoCo 反号
`+0.998203/-1.875899`。因此最终问题答案为否：上层 equilibrium correction 只关闭 static
compatibility，不能使原 repair 通过完整 H0 validation。

按 mandatory stop 未进入 REAL/SHORT/10 s/REAUDIT。Phase45 继续 REWORK；Phase46 tracking
继续不获授权。
