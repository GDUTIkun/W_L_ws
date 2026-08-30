# Phase 43 Repair Selection

正式分类：

```text
P43-U_none_of_the_frozen_minimal_candidates_satisfies_all_gates
```

没有结构进入minimum-complexity比较，因为A/B/C/D均未满足共同mandatory gates。特别问题的答案是：

```text
在当前task normalization与三档冻结bandwidth下，
xi-hold realization不足以稳定native wheel-rate mode；
但简单叠加独立native wheel-rate damping同样不足。
```

证据不支持“D只需继续加gain”或“恢复A+B”。下一工作不能直接进入Phase44 tracking；必须新建
decision Phase，先解释为什么WBC task rows显著改善`ddxi/xi`却仍留下约`-3.09 rad/s²`的right native
wheel acceleration，并区分 reduced-controller realization与actual contact-constrained plant wheel row。
在该技术决策冻结前，不批准第五候选、改task weight、改wrench semantics或上12D NMPC。
