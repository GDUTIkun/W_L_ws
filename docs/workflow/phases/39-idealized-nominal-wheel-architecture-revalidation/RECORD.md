# Phase 39 RECORD

状态：`complete`  
日期：2026-08-30  
分类：`P39-D_x16_nonclosure_structurally_persists`  
H0：`P39-F_H0_spin_drift_persists`

Model B 以 append-only cylinder + centered radial COM 作为 ideal nominal validation plant；compiled
parity 证明相对 Model A 仅两轮 `body_ipos` X/Y 改为零。绝对 wheel-angle effect 降至
`6.045e-5 m/s²` 并通过，但 C1/C2/C3 仍为 `0.08480/2.07846/1.65810 m/s²`，所以 x16
non-closure 跨 configuration、velocity、wheel-rate 三族结构性保持。

Phase 35 H0 在 clean contact/resource gates 下仍由 right wheel 于 tick 96 穿越 live bound。
`±1 rad` 不再有 absolute-angle model-validity 依据，但 safety/domain contract 未获解除，production
gate 不变。下一 Phase 应先验证 long-horizon wheel-angle safety/observability/numerical representation，
之后才允许公平重开 Phase 34 tracking。

Authority：

- formal-v2：`evidence/automated/architecture-revalidation-formal-v2`；
- fresh replay-v2：`evidence/automated/architecture-revalidation-replay-v2`；
- summary SHA-256：`763cf09111a9fb664644a7b27b002cedfde1dc3fb3231f89ff5a32ffbc784c1a`；
- config SHA-256：`3cb175394dff64551efa4d3189235e171cff791b2da891d5b538a251f29d6f93`；
- Model B SHA-256：`407a7b6f227d137cffdeeae968831a27b3f356dd4e575a95637f342039a9cb3b`；
- MuJoCo 3.7.0 / NumPy 2.2.6 / SciPy 1.15.3 via repository `.venv`。

Review：[PASS](REVIEW.md)。Phase 34 tracking 未运行，production 未修改。
