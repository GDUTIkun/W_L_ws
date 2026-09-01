# Phase 48: Weighted-WBC / QP Realization Closure — PLAN

Status: `active`

> Phase 47 已完成且编号不可复用；本研究使用稳定 ID Phase 48。后续 NMPC candidate comparison
> 为 Phase 49。

## Primary Objective

在冻结 Controller Core、42D production Weighted-WBC、ROS/MuJoCo public boundary 和 current
MuJoCo plant 的前提下，用 fixed state 与 fixed/artificial `W_ref` 把

```text
W_ref → W_WBC → tau → W_MJ / state response
```

分成 hard realizability、weighted-task competition 和 plant realization 三层；对代表性
state/request 上的每个 material mismatch 给出可复现测量、机制分类和证据边界，并在动态与时序门
通过后冻结供 Phase 49 的 12X/16X candidates 共用的 lower-layer contract。Phase 关闭不要求
`W_ref = W_WBC = W_MJ`。

## Current State and Audit

- Phase 47 已完成 workspace cleanup/interface freeze；唯一 current runtime 是
  `Controller Core → ROS2 → MuJoCo`，public boundary 仅为 `RobotState/TorqueCommand`。
- `W_ref`、重建的 physical `W_WBC`、12D signed interaction-wrench slack 和 `W_MJ` 均为
  internal/regression diagnostics，不新增 ROS topic。
- current production QP 为 42D reduced decision
  `[nudot(12), tau(6), W_L(6), W_R(6), slack(12)]`。reduced/full constrained-dynamics equivalence
  已关闭；本 Phase 不重开 full-space、explicit `lambda_eq` 或 decision-layout 设计。
- corrected R1 已证明 aggregate wrench representation dynamics-sufficient；每轮 physical
  point-force image rank 为 5。R1、point-force/eta/lambda representation 不重开，除非 fresh
  regression FAIL。
- base-reference/force-dual `X_PM/X_MP` direction bug、W5 与 primitive contact-law W1–W6 已关闭；
  `Xdot*nu` general bias 必须保留，且在非零速度 state 做一次 covariance regression。
- authoritative fixed-H0 result：baseline normalized slack
  `0.001522220395389018`；primitive-R2 normalized slack `0.05850370867784012`，dominant right
  `Tx`；`W_ref` primitive-infeasible；minimum unavoidable normalized L∞ deviation
  `0.07832043067340007`。这是单一 witness，不是 feasible-region atlas。
- Phase 46 保持 `review/REWORK`：`COMP=PASS`、`EQ=FAIL`，`AUTH/REAL/SHORT/10 s=NOT ENTERED`。
  Phase 48 不覆盖、升级或重新解释该 verdict。
- current logging 已有 state、reference、42D solution、七类 task residual/cost、slack、raw/command
  torque、solver/hard residual 和 timestamps；`W_MJ` aggregate reconstruction、hard-feasibility
  verdict/closest wrench、constraint margins与 active-set signature需作为 Phase 48 evidence 补齐，
  但只在 diagnostic/formal 路径暴露。

## Scope

- 审计 `W_ref/W_WBC/W_MJ/tau/slack/residual` 的 frame、origin、actor/receiver、sign、order、unit
  与 same-tick provenance，并 fresh reproduce authoritative H0。
- 在少量代表性 fixed states 上构造分阶段 wrench basis，求 exact hard-feasibility 与 normalized
  L∞ closest-feasible wrench。
- 只对选定 hard-infeasible cases 做 hard-limit attribution；只对已证明 hard-feasible 且
  `W_ref→W_WBC` material 的 cases 做 weighted-task/KKT attribution。
- 对 representative cases 测量 `W_WBC→tau→W_MJ`，再做少量 state、active-set、dynamic 和 timing
  validation。
- 明确区分 implementation/semantic bug、normal hard unrealizability、valid task trade-off 与 normal
  constrained-plant mismatch；明确 lower-layer freeze 和 Phase 49 handoff。

## Non-Goals

- 不实现或比较 12X/16X NMPC，不提前选择 candidate。
- 不做 realization-aware NMPC、MPC residual feedback、feasible-wrench projection、RL/learned residual。
- 不重写 full-space QP、decision layout、explicit equality reaction、point-force decision或 contact
  representation；不做 global weight tuning/optimization。
- 不研究 hardware、terrain、sim-to-real 或完整 friction/compliance solver decomposition。
- 不建立完整 feasible-wrench/state/active-set atlas，不跑 state × axis × side/mode × magnitude ×
  ablation 的笛卡尔积。

## Frozen Decisions and Interfaces

- Model/profile/solver：current axisymmetric centered-COM MuJoCo model、current nominal Weighted-WBC
  profile、ProxQP production solver及 current hard constraints；每个 formal run 的 manifest 必须记录
  model/config/controller revision、solver options、seed、threshold、input hashes、解释器和依赖版本。
- `W_ref/W_WBC/W_MJ` canonical order：
  `[L_Fx,L_Fy,L_Fz,L_Tx,L_Ty,L_Tz,R_Fx,R_Fy,R_Fz,R_Tx,R_Ty,R_Tz]`；controller-body FLU；
  moment about corresponding wheel-body origin；wheel follower-on-leg/base；force N、moment N·m。
- `W_WBC` 必须从 authoritative physical solution 经 production interaction map 重建，不使用 latent
  null component。12D slack只属于 interaction-wrench fidelity，并保持
  `W_WBC - W_ref - signed_slack = wrench_residual`；不得发明 contact/leg slack。
- `W_MJ` 必须由 same-tick MuJoCo contact/efc reaction → Cartesian point forces → per-wheel aggregate
  → canonical production reference 重建；不进入 public feedback。
- public `RobotState/TorqueCommand`、joint order、adapter torque sign、saturation、100 Hz WBC、500 Hz
  nominal physics、reset/ZOH/watchdog contract保持不变。
- Phase 46 production reduced-QP validity、corrected R1、W5、primitive W1–W6 与 torque-replay结论只作
  regression oracle。任何 regression FAIL 先停止并分类，不直接扩大研究对象。

## Definitions and Threshold Freeze Task

所有数值阈值在首次 formal run 前由 `P48-T01` 从现有 solver/test contracts 与 Phase46 authority
冻结到 evidence method，之后不得按结果调阈值。至少定义：

- per-channel scale `s=[50,50,50,2.5,2.5,2.5]` per side；
  `||e||∞,norm = max_i |e_i|/s_i`。
- hard-feasible：在同一 42D variables、hard equalities/inequalities、primitive operator和 bounds下，
  增加 `W_realized=W_ref` 后 solver/status/residual/margin 同时满足冻结 tolerance。
- closest hard-feasible wrench：在同一 hard set 上最小化 epigraph `t`，约束
  `-t s <= W_realized-W_ref <= t s`；报告 lexicographic tie-break（先 `t`，再最小 normalized L2）
  以保证可复现，不把 production soft objective混入 feasibility verdict。
- material mismatch：按预冻结 absolute + normalized tolerance 双门判断；低于门则不做深归因。
- active constraint：按预冻结 bound margin/dual tolerance 判定；同时保留 raw margin/dual，避免仅靠
  二值标签解释机制。

## Bug Taxonomy

| Code | Classification | Bug? |
| --- | --- | --- |
| BUG-A | frame/origin/sign/order/actor/tick/state interface semantic error | yes |
| BUG-B | QP row/block/scale/slack/cost/bound assembly error | yes |
| NORMAL-C | `W_ref` outside hard-feasible set | no |
| NORMAL-D | hard-feasible reference被 valid weighted objective主动让渡 | no |
| BUG-E | wrong task semantic/reference/sign/bias/subspace/normalization | yes |
| NORMAL-F | correct constrained plant/contact response导致 `W_WBC != W_MJ` | no |
| BUG-G | `W_MJ` reconstruction/contact grouping/frame/time alignment error | yes |

“dominant contribution”不等于 bug，也不等于 repair authorization。

## Task Graph

```text
P48-T01 semantics + evidence contract
    ↓ G0
P48-T02 fresh H0 baseline regression
    ↓ G1
P48-T03 fixed-H0 hard-realizability screen
    ├─ infeasible representatives → P48-T04 hard-limit attribution
    └─ feasible + material ref→WBC → P48-T05 task competition/semantic audit
                         └─ no material mismatch → skip deep attribution
    ↓ G2 (T03 mandatory; T04/T05 conditional cases closed)
P48-T06 WBC→plant fixed-state realization
    └─ material mismatch only → conditional plant attribution
    ↓ G3
P48-T07 representative state + active-set robustness
    ↓ G4
P48-T08 dynamic lower-layer + timing/replay validation
    ↓ G5
P48-T09 final review inputs + lower-layer freeze/handoff
```

## Tasks

| ID | Task | Input / Dependency | Deliverable | Required Validation / Evidence | Status |
| --- | --- | --- | --- | --- | --- |
| P48-T01 | Freeze semantics, method, thresholds and schema | Phase47 interface freeze; Phase46 authority; current source/logging | `REALIZATION_METHOD.md`; evidence schema; minimal diagnostic implementation plan | Static provenance table; independent identity checks; manifest schema validation; no experiment | done |
| P48-T02 | Fresh baseline/semantic integrity regression | T01 / G0 | append-only H0 formal + fresh replay evidence; `QP_REALIZABILITY.md` baseline section | exact canonical mappings; same-tick IDs; H0 authoritative values within frozen tolerance; reduced-QP/R1/W5/W1–W6/COMP regressions PASS | done |
| P48-T03 | Fixed-H0 hard-realizability screen | T02 / G1 | request catalogue; exact verdict, closest wrench, min deviation, dominant component, active constraints/margins per case | independent feasibility/minimax solve; solver cross-check or primal witness; deterministic replay; no production soft costs in verdict | done |
| P48-T04 | Conditional hard-infeasibility attribution | only selected infeasible T03 cases | `QP_REALIZABILITY.md` attribution sections | nested relaxation/constraint-removal diagnostics with witness closure; classify structural subspace vs active bound vs semantic bug | done |
| P48-T05 | Conditional weighted-task and task-semantic audit | only exact-feasible T03 cases with material `W_ref→W_WBC` | `TASK_COMPETITION.md` | per-task normalized residual/cost; total objective closure; fixed-active-set KKT/projected-gradient attribution; one-task diagnostic ablation only when needed; restore baseline after every ablation | todo |
| P48-T06 | Fixed-state WBC-to-plant realization | G2; representative feasible/infeasible cases | `WBC_PLANT_REALIZATION.md`; same-tick structured probes | record `W_ref,W_WBC,tau,primitive prediction,W_MJ`; close three error identities; torque replay regression; conditional wheel/component attribution only for material mismatch | todo |
| P48-T07 | Minimal state/configuration and active-set robustness | T06 / G3 | state delta set; transition evidence; covariance regression | H0 plus one pose/configuration, one nonzero-rate/forward state, one left/right-asymmetric state; `Xdot*nu != 0` regression; per-tick refresh/signature checks across selected transition | todo |
| P48-T08 | Dynamic lower-layer and timing validation | T07 / G4 | `LOWER_LAYER_RUNTIME.md`; short/long formal; fresh deterministic replay | ordered entry: standing, small ±forward/common/differential only as supported; 223 ticks before 1000 ticks/10 s; state/wrench/solver/margin/signature logs; dt/age/ZOH/reset/deadline/miss-policy gates | todo |
| P48-T09 | Review package and Phase49 handoff freeze | T01–T08 / G5 | REVIEW inputs and, only after REVIEW PASS, lower-layer freeze updates/RECORD | zero blocking findings; evidence hashes resolve; public boundary unchanged; 12X/16X consume identical frozen `W_ref` contract/current path | todo |

任务状态只使用 `todo / doing / done / blocked`。本 planning turn 不开始 P48-T01。

## Minimal Request and State Design

### Stage 1 — H0 screening basis

避免把 nominal request 与 axis/mode全组合。先运行：

1. authoritative nominal H0；
2. bilateral common `±Fx, ±Fz, ±Ty`；
3. bilateral differential `±Fy, ±Tx, ±Tz`；
4. one left-only and one right-only request，优先使用 Stage 1 中最不对称或最接近边界的 component；
5. 每个 direction先单一 small magnitude；只有 verdict边界、非线性或 active-set变化时增加一个
   half/full magnitude pair。

这提供所有六个 wrench component、common/differential与 side asymmetry 的小代表 basis，而不是
`6 axes × 2 signs × 3 modes × N magnitudes`。exact-feasible 样本不足以支撑 T05 时，只沿最有 margin
的已有方向缩小 magnitude，最多二分到预冻结次数；不新增任意方向搜索。

### Stage 2 — conditional deep attribution

- hard-infeasible：仅选 nominal H0、一个 dominant single-axis、一个 common/differential代表；
- hard-feasible但 WBC nearly exact：记录后停止，不做 ablation；
- hard-feasible且 material mismatch：只对最大 weighted contribution及最多一个对照 task做 KKT/ablation；
- `W_WBC→W_MJ` non-material：记录后停止，不做 plant mechanism decomposition。

### Stage 3 — state dependence

只保留 H0 加三个 delta states：一个 pose/joint configuration、一个 `qdot`/small forward velocity、
一个左右不对称 state。每个 state 复用 nominal + 两个由 H0 选出的最有信息 request；只有 classification
改变才追加一个 magnitude。不得构造 atlas。

## Mandatory Gate Order

| Gate | Pass condition | Failure action |
| --- | --- | --- |
| G0 SEMANTICS | canonical mapping/provenance、normalization、solver/active definitions冻结且 identity检查可执行 | BUG-A/G或证据设计缺口；最小修复/补设计后重跑，不进入 H0 |
| G1 BASELINE | fresh H0复现；R1/W5/W1–W6/42D/COMP无 regression；Phase46 verdict保持原样 | regression stop；只修明确 bug，不进入新 characterization |
| G2 HARD/TASK | T03 basis完成；每个 infeasible representative有 hard attribution；task attribution只来自 exact-feasible cases | 未分类 case阻塞；禁止用 hard-infeasible reference解释 pure task competition |
| G3 PLANT | three error vectors、same-tick provenance和 primitive/MuJoCo reconstruction闭合；material gap已分类 | BUG-G先修；architecture-dependent机制进入 decision stop |
| G4 STATE/ACTIVE | selected states与 transition中 operators/rows每 tick刷新；`Xdot*nu` covariance PASS；无未知 classification migration | robustness或semantic failure停止，不进入 rollout |
| G5 DYNAMIC/TIMING | 223-tick后才进入1000-tick/10 s；safety/contact/solver/timing/replay gates均 PASS | 首个 mandatory FAIL 有序停止，不运行更长或更复杂 reference |
| G6 FREEZE | REVIEW=PASS、zero blocking findings、contract/evidence hashes完整 | REWORK；不创建 RECORD，不授权 Phase49 |

## Conditional Decision Rules

```text
W_ref exact hard-feasible?
├── NO  → minimize deviation → selected hard-limit attribution → skip pure task competition
└── YES → compare W_ref and W_WBC
          ├── non-material → record and stop attribution
          └── material     → KKT/task competition → semantic audit

W_WBC vs W_MJ material?
├── NO  → record closure; no deep plant attribution
└── YES → validate reconstruction first
          ├── invalid → BUG-G minimal fix + regression
          └── valid   → bounded wheel/component/state/active-set attribution
```

若 solver `unknown/inaccurate/nonfinite`、witness不闭合或独立 replay不一致，不得把结果分类为
hard-infeasible、task trade-off或 plant mechanism。

## Evidence Design

正式 evidence 使用 append-only run directory，至少包含：

- `manifest.json`：Phase/task/run ID、UTC/local date、git commit+dirty files、model/config/controller hashes、
  `.venv` interpreter、MuJoCo/NumPy/SciPy/solver versions、seed、dt、solver settings、thresholds、
  input catalogue hash、command和 `supersedes`（若有）；
- `summary.json`：gate verdicts、taxonomy、case counts、worst metrics、first failure、skipped conditional
  branches及原因；
- `probes.csv`（或按层拆分但共享 schema/version）：case/state/tick/substep/sample/command timestamps，
  contact/active-set signature，state snapshot，`W_ref/W_WBC/W_MJ`，slack/residual，tau，primitive
  predicted reaction，task residual/cost，hard-feasible verdict/min deviation/closest wrench，constraint
  margins/duals，solver status/residual/time；
- `replay-summary.json`：fresh-process semantic/numeric parity、input/output hashes和 first mismatch。

长期字段必须逐 component显式命名或附 schema，不允许仅用无解释的 `v0...vN`。所有 mismatch同时保存
raw physical unit与 normalized value。每个跳过的 conditional branch也写 machine-readable reason。

正式输出前必须按仓库规则用 `./.venv/bin/python` 做依赖探针并记录版本，再 `py_compile`；失败不得创建
或污染 formal output directory。代码/runner实现阶段从 `ros_ws/` 执行 targeted `colcon build`。

## Reuse vs New Evidence

以下已由 Phase46关闭，只在 P48-T02 fresh regression，不从零 attribution：

- 42D reduced-QP/full-dynamics equivalence和 decision layout；
- corrected rank-5 R1/dynamics sufficiency和 production-reference map；
- `X_PM/X_MP` force-dual direction与 W5；
- primitive contact law W1–W6、42D H0 witness与 COMP；
- H0 slack identity、primitive infeasibility/minimax witness；
- QP wrench不是 plant direct input、same-state torque replay能复现 actual reaction。

新 evidence只覆盖：代表性 request/state 的 hard region samples；选定 infeasible mechanisms；
hard-feasible references上的 production weighted competition；统一 same-tick三层 error；状态/active-set迁移；
通过前述门后才做 dynamic/timing closure。

## Stop Conditions

- 任一 semantic/provenance/reconstruction identity FAIL：停止下游，按 BUG-A/G处理。
- Phase46 frozen regression FAIL或 H0无法 fresh reproduce：停止新实验；不得重解释历史证据。
- hard-feasibility solver无可信 witness、independent solve不一致或 tolerance未预冻结：状态 `blocked`，
  不作物理不可达结论。
- hard-infeasible reference：该 case禁止进入 T05；只走 T04。
- architecture/model选择才可解决的问题（task hierarchy/weights redesign、wrench/contact representation、
  NMPC/public interface、major physical model）：decision gate停止并请求新技术决策，不顺手修改。
- dynamic gate首次失败：停在当前长度/reference，不继续更长、更复杂或调 weight掩盖。
- evidence FAIL只说明相应 gate；解释器/依赖失败为 environment failure，不是模型/controller FAIL。
- 任何未解决 blocking finding：REVIEW=`REWORK`，不创建 RECORD，不把 Phase48/49标 complete。

## Bugfix and Tuning Policy

- clear BUG-A/B/E/G 可在本 Phase 按 `one bug → minimal fix → targeted regression → full affected-gate
  regression → resume`处理；保留用户/其他改动，不改无关区域。
- bugfix不得改变冻结 task architecture、weight hierarchy、wrench semantics、contact representation、
  NMPC interface或 major plant model；需要这些变化时停止到 decision gate。
- Phase 允许解释 current weights产生的 optimum，不允许 global tuning、automated weight optimization或用
  weight掩盖 hard infeasibility/semantic/plant mismatch。

## Phase 48 Close Criteria

- [ ] `SEMANTIC BUGS: NONE KNOWN`（在本 Phase 已审计范围内）。
- [ ] representative requests/states 的 hard realizability已分类；infeasible代表有 minimum deviation与机制解释。
- [ ] weighted-task competition仅在 exact hard-feasible references上完成，task semantics已审计。
- [ ] `W_ref→W_WBC`、`W_WBC→W_MJ`、`W_ref→W_MJ`均 same-tick 可测、可复现、可解释。
- [ ] basic state dependence与 selected active-set/contact transitions无未知 robustness finding。
- [ ] ordered dynamic lower-layer rollout、timing/reset/ZOH/deadline和 fresh replay全部 PASS。
- [ ] public boundary不变，internal diagnostics/schema与 lower-layer contract冻结。
- [ ] REVIEW=`PASS`且 blocking findings为零；之后才创建 RECORD并更新 ROADMAP为 `complete`。

## Handoff to Phase 49

Phase 49 的 12X 与 16X candidates必须只通过相同 `W_ref` contract接入同一个 frozen
Weighted-WBC/current ROS/MuJoCo path，并使用相同 model/profile/solver/timing/logging与 evidence schema。
Phase 48 不评价两 candidate优劣；历史 16X failure不作为 future fair comparison 的最终 verdict。

## Planned Documentation

- 本文件：任务、依赖、门、证据与 stop authority。
- `REALIZATION_METHOD.md`：P48-T01 semantics、normalization、problem definitions和 schema。
- `QP_REALIZABILITY.md`：P48-T02–T04 evidence interpretation。
- `TASK_COMPETITION.md`：P48-T05。
- `WBC_PLANT_REALIZATION.md`：P48-T06–T07。
- `LOWER_LAYER_RUNTIME.md`：P48-T08及 Phase49 contract。
- `REVIEW.md`：只在 execution进入 review时创建；`RECORD.md`只在 REVIEW PASS后创建。

## Planning-Turn State

```text
PHASE 48: ACTIVE
P48-T01 / P48-T02: DONE — G0/G1 PASS
P48-T03+: NOT STARTED
NEXT ACTION: execute P48-T03 fixed-H0 hard-realizability screen in a separate prompt
```
