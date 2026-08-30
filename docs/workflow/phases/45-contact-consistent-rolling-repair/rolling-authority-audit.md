# Rolling authority audit

正式 v3 未进入：`DG45-EQ` 先失败，`directional-authority.csv/json`明确记录`entered=false`。

`contact-consistent-formal-v1`曾在发现EQ FAIL后错误继续计算directional probes，违反冻结停止顺序，
故完整保留但标为 rejected/non-authoritative，不用于Phase45结论。其数值不能覆盖v3的未进入事实。
