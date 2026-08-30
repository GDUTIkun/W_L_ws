# Phase 43 Instantaneous Screening

authority：`rolling-repair-formal-v3/snapshot-audit.csv`；fresh replay逐字段一致。

DG43-BASELINE PASS：R43-0 精确复现 tick111 first right contact loss，双运行除`wbc_time_s`外
semantic error为0。fixed-state oracle的最大 whole-vector dynamics residual为`7.11e-14`，contact
reconstruction residual为0。

tick0 关键结果：

| Candidate | `ddxi_common` m/s² | `ddxi_diff` m/s² | native wheel qdd L/R rad/s² | DG43-EQ |
| --- | ---: | ---: | ---: | --- |
| baseline | -0.120206 | -0.021445 | -0.219552 / -3.210043 | reference |
| A | +1.540957 | -0.022137 | +39.242190 / +36.332706 | FAIL |
| B, all gains | -0.031290 | -0.021529 | -0.094340 / -3.086199 | FAIL |
| C, all gains | -0.031863 | -0.021530 | -0.094965 / -3.086849 | FAIL |
| D, all gains | -0.031864 | -0.021530 | -0.094961 / -3.086851 | FAIL |

B/C/D 在零初始 reference error/rate时 bandwidth不改变tick0 target，故三档结果相同。它们把
common `ddxi`压到0.05 m/s²以内，但 right native wheel qdd仍约`-3.09 rad/s²`，超过冻结1.0门；
不能把`dxi`改善误写成native-rate equilibrium。A 的 core-side trim在实际 MuJoCo plant oracle上
反而放大common acceleration，因此mechanism FAIL。

ticks 46/74/101/110只保留为旧轨迹local stress diagnostics，不用来单独否定新闭环。
