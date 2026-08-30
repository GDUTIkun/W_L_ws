# Phase 40: Wheel Absolute-Angle Domain / Representation Contract Validation — PLAN

状态：`complete`  
日期：2026-08-30

## 审核结论

用户草案方向获批，但按以下边界收紧后执行：

1. Phase 40 可裁决 Model B、当前 C++ controller/WBC 和仓库内实验性测量链的 angle contract；
   真机仍冻结，不能由“仓库未见限位”推断真实装配无 cable、hard-stop 或 sensor limit。
2. `P40-F` 只能表示当前 `±1 rad` 缺乏已验证的 model/software/mechanical authority，不在本
   Phase 修改 production gate。
3. raw wrapped state 不等同于 periodic physical evaluation；所有普通 subtraction consumer 必须
   单独审计。
4. shadow H0 允许使用显式 diagnostic-only wheel-workspace policy 继续越过历史 gate；默认
   controller/WBC API 行为和原 gate 必须由 regression 证明不变。
5. Phase 34 tracking、gain/task/controller architecture 和 hardware schema 均不在本 Phase 修改。

## 唯一目标

为 absolute wheel joint angle 冻结一个长期表示契约，并回答当前 `[-1,+1] rad` gate 是否仍有
nominal model、数值、controller/estimator 或已验证 safety 依据。

## Grounding authority

- Phase 39：`P39-D + P39-F`；Model B absolute-angle family PASS，H0 tick 96 由 right wheel
  触发 live bound。
- CBM project `W_L_ws`，generation `2026-08-29T06:47:42Z`，9580 nodes / 17210 edges；
  `nominal_wbc_model.*` 等已有修改路径直接读源码补齐 freshness。
- Graphify 现有图用于 Phase 21/35 workspace 历史关系，不替代当前源码或 formal evidence。
- live consumer 初查：MuJoCo Adapter 传递 unwrapped qpos；NominalWbcModel 对 raw `q-qeq`
  应用 `[-1,+1]` wheel bound；WBC physical kinematics 使用 periodic `AngleAxis`；x16 NMPC 不消费
  wheel absolute angle；实验性 STM32 C620 wheel path以 `Total_Round/Total_Encoder` 重建 multi-turn
  angle，但正式 hardware/estimation contract 尚未冻结。

## Frozen representations

- R0：raw unwrapped plant coordinate `q_u`，velocity 独立测量/传递。
- R1：`wrapToPi(q_u)`；仅作为候选，不允许直接用于普通 position residual。
- R2：`q_r = q_u - 2πk` + separate revolution count；recenter 必须 physical-equivalent 且不改 dq。
- R3：raw unwrapped plant + periodic physical validator；absolute magnitude 不作为 model-validity
  条件，accumulated rotation 需要时单独保留。

formal 前不预选 winner。

## Strictly forbidden

- 不修改 x12/x16、NMPC、planner、WBC task/gain/wrench/torque/contact/model parameters；
- 不运行 Phase 34；不宣布 tracking PASS；
- 不删除或放宽 production gate；diagnostic policy 不得成为默认路径；
- 不把实验 UART2/C620 实现升级为已批准 hardware contract；
- 不把 wheel absolute angle 与 xi 或 wheel spin rate 合并解释。

## Gates

### DG40-00 — consumer / measurement / safety semantics

枚举所有 live q/dq consumers，按 A–J 分类并记录 raw subtraction、periodicity、near-equilibrium、
wrap discontinuity 和 revolution-count requirement。审计 MuJoCo、RobotState、ROS、STM32 bridge、
实验 firmware；找不到真实 safety authority 必须写为 unknown/not established，而非证明不存在。

无法完整追踪 live source boundary：`P40-U`，停止。

### DG40-01 — frozen corpus and thresholds

primary Model B。authority state 使用 Phase 32/39 T0 frozen plant state 与 fixed baseline torque。
对 left/right/bilateral mode 测试：

```text
k mandatory = 0, ±1, ±5, ±25, ±50 revolutions
k engineering extension = ±500, ±5e3, ±5e4, ±5e5, ±1e6
k diagnostic-only extension = ±5e6, ±5e7, ±5e8
q = q0 + 2πk
```

工程 horizon 在看到结果前冻结为 `|k| <= 1e6`。diagnostic-only extension 只定位 double
precision 退化尺度，不把任意极大浮点角反推成 `±1 rad` gate。

冻结 gates：mandatory/engineering physical geometry、transform、contact、M/bias/J/qacc/ddxi/load
相对 modulo authority的 absolute/normalized error均须 `<=1e-8`（量纲量同时完整报告 raw error）；
material dynamic error `<=1e-4 m/s²`，rotation orthogonality `<=1e-10`，contact topology exact，
finite required。finite-difference epsilon `1e-6 rad`，并报告 step/ULP 比；若 step 无法表示则明确
记为 large-angle derivative precision limit。

### DG40-02 — long-horizon periodicity and precision growth

报告 wheel center、xi/zeta、body transforms、contact geometry/topology/load、M/bias/J、closure、
qacc、physical ddxi、wheel angular acceleration、condition、finite 和 error-vs-|k|。mandatory 或
engineering horizon 失效才构成 nominal numerical-domain finding；更远 extension 只报告 first
material scale。

### DG40-03 — wrapped and recentered audit

R1 在 `±π` 的 `π±1e-6` 检查 physical continuity 与 raw software residual jump。R2 对每个 corpus
state执行 exact `2πk` recenter，验证 physical outputs、dq、contact、model matrices和 dynamic
response parity；revolution count 与 local angle 必须可逆重建 raw q。

### DG40-04 — representation classification

- `P40-A`：physical coordinate cyclic/unbounded，工程 horizon 内 R0 数值健康；
- `P40-B`：工程 horizon 内出现 material unwrapped numerical degradation且 R2 restores parity；
- `P40-C`：只有 raw wrapped representation满足 contract；
- `P40-D`：具体 controller/estimator consumer要求有限 local domain；
- `P40-E`：有已验证 mechanical/sensor finite limit；
- `P40-F`：当前 `±1 rad` contract 无支持；
- `P40-U`：authority/validity不足。

组合允许。hardware unknown 不能映射为 `P40-E`，也不能阻止 nominal/software `P40-F`，但会阻止
直接 production removal。

### DG40-05 — shadow H0

仅 DG40-00～04 形成有效候选后运行：Model B、Phase27 Minimal、same fixed equilibrium wrench、
no xi task/no target。raw plant q 保持 unwrapped；production default gate 不变；diagnostic-only
policy仅忽略 wheel absolute-magnitude workspace rejection，leg bounds 与所有 model/solver/safety
gates继续生效。

预冻结终止：最多 `2000 ticks / 20 s`，达到任一 wheel `|q-q0| >= 6π`（3 revolutions）或出现
contact、finite、model conditioning、WBC hard/slack/torque、base envelope 等独立 failure 即停。
不得把通过 shadow rollout 写成 production-safe。

## Tasks

| ID | Task | Deliverable | Status |
| --- | --- | --- | --- |
| P40-T01 | consumer/source trace | `wheel-angle-consumer-audit.md` | done |
| P40-T02 | measurement/estimation audit | `measurement-estimation-contract.md` | done |
| P40-T03 | safety authority audit | `safety-domain-audit.md` | done |
| P40-T04 | freeze config/corpus | append-only config | done |
| P40-T05 | implement offline periodicity/numerics runner | Python runner | done |
| P40-T06 | implement diagnostic-only shadow H0 policy/entry | minimal C++/runner | done |
| P40-T07 | formal + fresh replay | stable evidence | done |
| P40-T08 | representation verdict | contract/audit docs | done |
| P40-T09 | REVIEW | `REVIEW.md` | done / PASS |
| P40-T10 | RECORD only after PASS | `RECORD.md` | done |

## Required deliverables

`PLAN.md`、`wheel-angle-consumer-audit.md`、`representation-contract.md`、
`long-horizon-periodicity-audit.md`、`large-angle-numerics-audit.md`、
`wrapped-discontinuity-audit.md`、`recenter-equivalence-audit.md`、
`measurement-estimation-contract.md`、`safety-domain-audit.md`、`shadow-h0-long-horizon.md`、
`workspace-contract-proposal.md`、`REVIEW.md`；REVIEW PASS 后才创建 `RECORD.md`。

## Verification protocol

- repository Python only：`./.venv/bin/python`；stable output 前 dependency probe + `py_compile`；
- C++ 变化必须从 `ros_ws/` 执行 targeted `colcon build` 与相关 tests；
- default gate behavior regression 必须 PASS；
- formal 与 fresh replay 独立目录，decision summaries semantic-equal；
- manifest 记录 interpreter/dependencies/executable/config/model/source/authority hashes；
- JSON/XML parse、evidence schema、`git diff --check` 为 REVIEW 前置项。

## REVIEW must answer

用户草案第 20 节全部 12 个问题；所有答案必须分别限定 nominal model/software、experimental
measurement path 和 unresolved real hardware authority。
