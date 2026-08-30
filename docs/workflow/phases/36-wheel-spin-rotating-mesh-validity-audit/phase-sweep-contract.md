# Phase sweep contract

冻结配置：`simulation/mujoco/config/phase36_wheel_phase_validity_v1.json`。

- coarse：`0, ±0.25, ±0.50, ±0.75, ±1.00, ±1.25, ±1.50, ±2.00 rad`；
- boundary：`±0.95, ±0.99, ±1.00, ±1.01, ±1.05, ±1.10 rad`；
- periodic：base `-2,-1,0,1,2 rad` 分别与 `+2π` 配对；
- 每组均执行 left-only、right-only、bilateral；
- 在读 formal 结果前搜索 mesh 的 2/3/4/5/6/8/10/12/16/20/24 阶精确有限旋转对称。
  只有 Hausdorff 距离不超过 `1e-6 m` 才增加 `2π/n` 等价候选；否则不事后挑 phase。

判据同时覆盖 finite、continuity、periodicity、phase sensitivity、contact topology 与瞬时动力学。
periodic absolute/relative gate 均为 `1e-8`；wheel-origin invariance 为 `1e-10 m`；
material effect 为 contact point 改变 `≥1e-4 m` 或 physical ddxi 改变 `≥0.05 m/s²`。
`±1` boundary special 要求局部 jump 相对邻近区间至少 `5×` 且 ddxi jump `≥0.05 m/s²`；
仅有正常的 piecewise contact topology change 不自动证明 `1 rad` 是自然边界。
