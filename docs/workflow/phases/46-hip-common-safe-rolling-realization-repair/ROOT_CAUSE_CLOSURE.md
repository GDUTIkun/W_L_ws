# Root-cause closure

## Verdict

- torque generation：`T2-ACCELERATION_TASK_COUPLING_DOMINANT`；
- first material mismatch：`R1-AGGREGATE_POINT_REALIZABILITY_MISMATCH`；
- Phase 状态：`review/REWORK`；本轮没有实施 repair。

## Full chain

xi/rolling task target 经 fixed-active-set KKT 生成 Fn-labelled QP solution direction；该方向的
`Δtau` 主要是 acceleration/task-driven bilateral hip/knee torque。torque 在 solver 之前造成
bilateral rolling free acceleration。QP 同时使用 aggregate 6D wheel wrench 中 actual two-point
contacts 无法实现的 `Ml` direction；该分量承担约全部 QP-predicted rolling cancellation。

plant 不接收 QP wrench，只接收 `Δtau`。MuJoCo contact solver 因此对已有 free-motion tendency
产生相反的 constrained reaction，形成 actual `Fn→Fr` 与 slip→xi harmful coupling。solver
reaction 的左右 cross ratio 在 free acceleration 中已经基本形成，所以不是 solver 独立把 normal
force 转成 rolling force。

R3 不成立为 first mismatch：KKT 证明 task 是 excitation source，但没有证据表明 task block 在
aggregate contact model 内部算错；首个明确违反 actual plant admissible set 的位置是 aggregate
wrench 到 two-point force image 的 rank-5 realizability boundary。

证据：[causal chain](evidence/automated/root-cause-closure-formal-v3/fn-fr-causal-chain.json)、
[decision](evidence/automated/root-cause-closure-formal-v3/root-cause-decision.json)。
