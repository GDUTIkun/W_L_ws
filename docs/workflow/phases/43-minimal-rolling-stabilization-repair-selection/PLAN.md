# Phase 43: Minimal Rolling Stabilization Repair Selection — PLAN

状态：`review`  
日期：2026-08-30

## 审计结论

用户草案获批执行，但补齐以下冻结定义后才可形成可证伪实验：

1. `xi` 是轮心相对 base 的位置，改 absolute wheel angle 不会产生 `xi` 位移；P43-3/4 因此冻结为
   50 ms、2 N 的轮体水平外力 common/differential pulse，并从 pulse 完全撤除后的状态评价恢复。
2. B 的 native-rate task 明确为 reduced acceleration 的 wheel joint rows 8/11；canonical 与 native
   joint sign 相反，但在 `qdd_des=-K qdot` 中符号变换抵消。审计仍同时报告 raw native rate与
   direction-normalized rim rate `v_r=-rho*qdot_native`，不把它等同于 `dxi`。
3. A 的 trim 只允许改变左右 `Fx`、`Ty` 四个 interaction-wrench 分量，使用冻结 bounds、正则和
   最大评价次数的 deterministic least-squares；不引入反馈，且完整记录 `Delta W_eq`。
4. 三档 gain 在 formal 前由 bandwidth 唯一换算：xi PD 使用 `Kp=omega^2`、`Kd=2*omega`
   (`zeta=1`)；native rate 使用 `Kd_rate=omega`。不得追加第四档或看结果后改 task scale。
5. late Phase42 snapshots 只作 local stress diagnostic；tick0 mechanism gate和新闭环轨迹才决定
   候选是否进入后续层。

## 目标与边界

在 Phase42 `P42-E_multiple_coupled_causes` authority 上选择满足全部 mandatory gates 的最小
WBC-side wheel-realization structure。Model B、Phase27 fixed wrench baseline、12D reduced
dynamics、contact/friction、torque limits、planner/NMPC 与 Phase34 tracking 全部冻结。

候选仅为：

- R43-A：四维 rolling-equilibrium wrench trim，无 feedback；
- R43-B：两行 native wheel-rate damping，无 xi task；
- R43-C：既有两行 xi-hold PD realization，无 planner/step/ramp；
- R43-D：C+B，且两组 task row独立保留。

不得增加 A+B、绝对轮角 task、第五候选或临时增益。

## Grounding 与接口冻结

- CBM project `W_L_ws`，generation `2026-08-29T06:47:42Z`；graph 指向
  `WeightedWbcProblem::assemble`、`WeightedWbcController::step`、
  `NominalWbcModel::evaluate` 和 Phase34 loop。相关 Phase42/tools/docs 未完整索引，直接源码与
  formal evidence 为 authority。
- Graphify 历史图确认 Phase21 rolling-contact representation、42-variable weighted problem和
  `WbcReference` 是既有演化路径，但 Phase42 尚未入历史图，不替代当前证据。
- QP 决策前12维为 reduced generalized acceleration；wheel canonical rows固定为8/11。
  B/D 新 reference 是 `wheel_joint_acceleration_rad_s2[2]`，task scale固定20 rad/s²。
- C 复用 `wheel_longitudinal_acceleration_m_s2` 与 Phase34 profile的 affine xi task；本 Phase
  reference恒为初始 `xi`，不创建或调用 planner target。
- Phase42 native authority精确恢复 ticks 0/46/74/101/110 的 full `qpos/qvel/time`；候选重新求
  WBC command，plant post-command量由 `mj_forward` oracle审计。

## 实验顺序与 gates

| ID | 内容 | PASS 条件 | 状态 |
| --- | --- | --- | --- |
| P43-T01 / DG43-BASELINE | baseline/provenance | R43-0 首失效精确为 tick111 right contact loss，hash与no-overwrite contract有效 | done |
| P43-T02 | candidate diff/component | A只改4个request分量；B只加native rows；C只加xi rows；D等于B+C；row/sign与non-finite测试通过 | done |
| P43-T03 / DG43-EQ | fixed-state screening | tick0 `ddxi_c/delta` 同时低于0.05 m/s²或不高于baseline 50%，native wheel qdd均不高于1 rad/s²；late snapshots只报告 | failed |
| P43-T04 / DG43-CONTACT/RATE/BASE/WBC/WR | 10 s nominal regulation | bilateral contact连续；rate/xi/base/torque/slack/hard/solver/workspace/non-finite与wrench degradation全部过冻结阈值 | failed |
| P43-T05 / DG43-PERT/DIFF | 四类small perturbation | pulse撤除/初态后无超过2x增长，末1 s不高于评价起点；common/differential rate、xi与load链均审计 | not entered |
| P43-T06 | minimum selection | 所有候选同门比较；先最少结构，再优先不改wrench semantics、再优先复用xi realization | failed: P43-U |
| P43-T07 | formal/replay/regression | append-only formal与fresh replay semantic-equal；targeted build/tests、parse/nonfinite/diff-check通过 | done |
| P43-T08 | REVIEW | 无 blocking finding才PASS | REWORK |
| P43-T09 | RECORD | 仅REVIEW PASS后创建并更新ROADMAP | todo |

阈值、扰动、trim bounds、三档 bandwidth 与所有输入在
`simulation/mujoco/config/phase43_rolling_repair_v1.json` 中冻结。程序遇首个 independent failure
立即停止；10 s 存活不是单独 PASS。

## 选择与解释规则

每个 structure 选择通过全部 gates 的最低 bandwidth；A无 bandwidth。结构复杂度顺序不预设，
按 request semantics修改、独立状态、task/row/gain数量、wrench degradation、torque/slack逐项报告。
若复杂度接近，先选择不改上层 wrench semantics者，再选择未来12D本来需要的xi realization。

正式分类为 `P43-A/B/C/D/U`。只有 C 或 D 全门通过才可授权相应结构进入 Phase44 hold→step/ramp
tracking；本 Phase 不宣布12D architecture、hardware或general robustness PASS。

## Deliverables 与验证

交付 `candidate-definition.md`、`instantaneous-screening.md`、`nominal-regulation.md`、
`perturbation-recovery.md`、`wrench-realization-audit.md`、`repair-selection.md`、`REVIEW.md`，仅PASS
后交付`RECORD.md`。machine-readable输出包括config、manifest、candidate/snapshot/event/rollout/gate表。

formal前使用`./.venv/bin/python`探针MuJoCo/NumPy/SciPy并`py_compile`；从`ros_ws/`运行targeted
colcon build/test；检查whole-vector dynamics、CSV/JSON parse、non-finite、hash、fresh replay和
`git diff --check`。必须显式记录：Phase34 run=false、12D NMPC run=false、16D repair=false、
plant/contact modification=false。
