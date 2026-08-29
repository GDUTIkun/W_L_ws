# Phase 29: NMPC corrective-action root-cause audit — PLAN

Status: `complete`

Design source: [NMPC Corrective-Action Root-Cause Audit](../../../mujoco/NMPC%20Corrective-Action%20Root-Cause%20Audit.md).
本 PLAN 接受设计稿的目标、范围和 A～G 诊断方向，但补充 solver lifecycle、精确目标函数、
非光滑灵敏度、交互项、constraint causality 与 full-solve oracle 的冻结合同。设计稿与本
PLAN 冲突时，以本 PLAN 为准。

## Goal

在不修改 production NMPC/WBC 控制律、不调参且不把 Phase 28 的冷启动 frozen-state
结果误当成闭环 solver lifecycle 的前提下，对 T0 static 与 T1 straight 首偏离附近的
非恢复性 interaction wrench 分别给出唯一、可复现的 NMPC 内部 primary root cause；
不能唯一隔离时明确输出 `unresolved/REWORK`，不选择“最可能”类别。

## Design Audit

设计稿的总体判断正确：Phase 28 已把 T0/T1 的首层问题关闭到 NMPC requested action，
因此本 Phase 应从 state/reference、cost、generated dynamics、OCP trade-off、constraints、
horizon 和 SQP-RTI lifecycle 内部继续追因，而不是回到 WBC、plant 或新增 stabilization
task。以下修正是执行前置条件：

- 原 A～G 改名为 `P29-A`～`P29-G`，避免与 Phase 28 的
  `B_nmpc_corrective_failure` 混用；后者是本 Phase 的输入层，不是本 Phase 的 B 类结论。
- 将 actual closed-loop solver lifecycle、每次 `reset()` 的 cold frozen solve、offline
  converged solve 分开。先逐 tick 复现 production request，之后的 counterfactual 才有
  authority。
- `J` 固定为当前 OCP 的完整有限域目标：全部 running stages 加 terminal stage，使用
  production `LINEAR_LS`、逐 stage advanced reference、当前 `Q/R/Qe`；不得以单时刻二次
  代价或另造 surrogate 代替。
- `du*/dx` 不是默认光滑量。扰动尺度、单边/中心差分、solver seed/lifecycle、收敛条件和
  active-set 稳定性必须预冻结；active set 改变时只报告 one-sided/nonsmooth response。
- “第一个单组清零后符号翻转”只能产生 candidate。必须结合 single-removal、成对交互、
  stage/adjoint 或等价 KKT 证据排除顺序依赖；贡献不可分时必须 REWORK。
- “约束活跃”不等于 `P29-F`。除 distance、multiplier 和 stationarity 外，还须使用只读、
  diagnostic-only 的 bound relaxation/deactivation shadow solve 或等价 KKT counterfactual
  证明该约束改变了 corrective direction；该操作绝不进入 production。
- full-solve oracle 使用 append-only、offline-only 的同构 SQP artifact 作为 primary；可用
  repeated-RTI iterate-carry 作 secondary cross-check。除 solver convergence mechanism 外，
  model、discretization、horizon、cost、reference、bounds、x0 必须逐项同一并保存 hash。
- 分类顺序固定为 semantics/cost/model 后先判定 RTI artifact，再诊断 constraint、cross-state
  和 horizon trade-off；后三者重叠且不能建立反事实隔离时不得强行唯一归类。

## Current State

- 已有：[Phase 27](../27-theory-restored-minimal-wbc/RECORD.md) 冻结 current nominal plant、
  16-state Eq.(12) NMPC、wheel-centre internal interaction-wrench、Minimal WBC、
  `2/10/20 ms` schedule 和 generated acados artifact。
- 已有：[Phase 28](../28-minimal-closed-loop-drift-attribution/RECORD.md) 在 authoritative v5
  中把 T0/T1 唯一关闭到 `B_nmpc_corrective_failure`，排除 WBC realization/resource 与
  reduced-model-to-plant mismatch；T2 right 与 T1 路径一致、left 不一致，未形成 T2
  primary attribution。
- 已有：T0 frozen-state 导数为 `+118.153/+18.2632`；T1 局部导数为
  `-0.972159/-0.491522`，但 snapshot 净纵向加速度为 `-0.0118472 m/s^2`，与负 position/
  velocity error 同向。它们是待解释现象，不预先批准任何 P29-A～G 原因。
- 已有：current OCP config 为
  `simulation/mujoco/config/phase27_acados_ocp_v2.json`，SHA-256
  `c24159aebbd7b38380044319e9cfdb619d880b86f9050aedc086ab07aba5eadf`；generated
  manifest SHA-256 为
  `142e85e155bf8654caaa456ef111a5e215fa5b7a734e64f1d156f3e753dce848`。
- 已有：`WheelAwareNmpcSolver::solve` 写入各 stage 的 advanced reference、bounds、x0，
  调用 generated SQP-RTI，并在内部读取全 horizon `x/u` 做 audit；公开 Result 目前只有
  `u0` 和 aggregate residual/objective，尚不能形成 stagewise root-cause evidence。
- 关键缺口：Phase 28 C++ frozen oracle 对每次 perturbation 都先 `solver.reset()`；它验证
  cold frozen response，但没有证明该 response 等于 T0/T1 闭环前缀中真实 solver state
  产生的 request。
- Grounding：CBM project `W_L_ws`，generation `2026-08-29T06:47:42Z`。solver/model/
  controller/config 候选路径没有 recorded coverage issue；新 Phase 28 test 与 `tools/`
  按索引策略直接读取。Graphify 只用于确认历史 NMPC/OCP/controller 边界，live source、
  Phase 27/28 manifest 与 raw evidence 优先。

## Scope

- 冻结 Phase 27/28 authority、exact snapshot tick、输入、reference、solver artifact、依赖、
  model/config/schema 与 hash；所有 Phase 29 run 使用独立 namespace 和非覆盖输出根。
- 建立三条显式分离的求解证据链：actual closed-loop lifecycle replay、cold frozen-state
  solve、同构 converged offline solve，并先证明第一条逐 tick 复现 authoritative request。
- 对 T0/T1 建立 physical state、16-state ordering、frame/sign、relative-rotation-vector、
  rate、moving reference、wheel `xi/dxi` 与每个 horizon stage `yref` 的 golden contract。
- 审计 exact finite-horizon `LINEAR_LS` objective 的 state/input/terminal gradient，以及
  generated discrete dynamics和独立 C++/解析模型的 state/input sensitivity。
- 以 additive offline diagnostic 读取或导出 stagewise `x/u/yref/cost`、bounds、multipliers、
  residual、stationarity/active set；不得把诊断值回写 controller 或 torque command。
- 对 T0/T1 执行预冻结的 single-removal、pairwise interaction 和 state-group
  counterfactual；用 requested wrench、predicted acceleration、objective/KKT 和 stagewise
  证据建立因果链。
- 对 production SQP-RTI 与 converged OCP 进行同输入比较；在后者仍非恢复时，继续以
  diagnostic-only shadow solves 隔离 constraint、cross-state 与 horizon/reference/terminal
  机制。
- T0/T1 根因关闭后，才对 T2 left/right 各取少量 holdout snapshot 检查同一机制是否
  一致；T2 不产生新控制架构或独立 primary root cause。
- 输出 state/reference contract、T0/T1 root-cause reports、counterfactual corpus、model/
  cost sensitivity、stage/horizon、constraint causality、RTI-vs-converged 和 final matrix；
  REVIEW/RECORD 按工作流在执行和审查完成后产生。

## Out of Scope

- 修改或调节 `Q/R/Qe`、state/input weights、bounds、horizon、`Ts`、RK4 substeps、reference、
  planner、model parameters、solver family或 production warm-start/reset 策略。
- 新增 WBC pitch、base-X、rolling、height、leg、wheel 或其他 task；修改 42D decision、
  104 hard rows、ProxQP、contact/torque/workspace limits或新增 inner loop。
- 再诊断 WBC realization、contact representation、MuJoCo plant mismatch、fast low-level
  stabilization，或改写 Phase 27/28 的结论、threshold、manifest、REVIEW/RECORD/evidence。
- 把 offline SQP、bound relaxation、held-reference、terminal removal 或 counterfactual
  action 反馈到 production controller；这些只允许作为因果 oracle。
- T3 single-RTI stationarity robustness、turning architecture、identified/CAD profile、真机、
  Hardware Adapter、STM32/树莓派、terrain 或任何 hardware claim。
- 选择修复、redesign、retuning 或批准新 task。本 Phase 只定位根因；修复必须另立 Phase。

## Frozen Decisions

- **Authority/non-overwrite：** Phase 27 formal-v2、Phase 28 v5 和 fresh replay v6 是输入
  authority。Phase 29 method/config/schema、oracle artifact 和 evidence 使用新名称与空输出
  目录，保存 `source_run/replay_of/supersedes`；失败、无效和被替代 run 均保留。
- **Production invariant：** current nominal MuJoCo plant、16-state definition、Eq.(12)、
  physical parameters、wheel-centre wrench semantics、`20 ms/N=20/two 10 ms RK4`、
  production SQP-RTI+partial-condensing HPIPM、Minimal WBC 与 `2/10/20 ms` schedule 不变。
  production generated artifact 和 normal solver wrapper不得原地修改。
- **Three-solve semantics：** `production-lifecycle` 从工况起点连续 replay 到目标 tick，
  保留真实 controller update/reset/iterate history；`cold-snapshot` 每个样本显式 reset，复现
  Phase 28 oracle；`converged-offline` 从同一 OCP data 求高精度 stationary solution。三者
  不能混写为同一个“frozen solve”。
- **Exact OCP objective：**
  `J=1/2*sum(k=0..N-1) ||Vx*x_k+Vu*u_k-yref_k||^2_W +
  1/2*||Vx_e*x_N-yref_N||^2_We`，矩阵、缩放和 terminal multiplier均从冻结 config/generated
  artifact读取。所有 gradient、cost decomposition 和 counterfactual 都报告完整目标与
  stage/terminal组成。
- **Reference semantics：** stage references只能由 production `advanceReference` 产生并保存；
  held-reference等替代序列只能是带标签的 diagnostic shadow input，不能替代 authority。
- **Perturbation validity：** 在查看 primary counterfactual 结果前冻结 state group、正负
  perturbation magnitude、numeric tolerance、solver seed/reset/lifecycle、one-sided/central
  规则和repeat count。扰动不得跨 orientation chart、state/input bound或改变非目标数据。
- **Nonsmooth response：** central derivative只在两侧 solver status、active set和local
  solution branch一致时成立；否则报告单边 response、active-set transition 或
  `nonsmooth/unresolved`，不得用一个斜率代表 `du*/dx`。
- **Counterfactual causality：** state groups至少为 base longitudinal、attitude、wheel
  position/rate、other state和reference/terminal。执行 actual、single-removal、预冻结的
  pairwise interaction；贡献不假设可加。单组翻转只能成为 candidate，需由交互/KKT/
  adjoint或等价 stagewise证据关闭。
- **Offline SQP oracle：** primary full solve 是 append-only、offline-only 的同构 generated
  SQP artifact，允许的差异仅为 `nlp_solver_type=SQP` 及预冻结 convergence options；
  repeated-RTI iterate-carry只作 secondary check。artifact/config/codegen/dependency hash、
  tolerances、iteration cap和termination reason全部入 manifest。
- **Constraint causality：** 只有同时具备接近/活跃 bound、可信 multiplier/KKT 证据，且
  仅解除目标 bound 的 diagnostic shadow solve 可重复恢复 corrective direction，才可归
  `P29-F`。放宽量和顺序预冻结；这不构成修改 production bound 的建议或批准。
- **Classification order：** 每个 T0/T1 snapshot 按
  `P29-A semantics -> P29-B exact cost direction -> P29-C model/control sign ->
  P29-G lifecycle/RTI artifact -> P29-F constraint -> P29-D cross-state coupling ->
  P29-E horizon/reference propagation` 执行。后一类只有在前类排除后才能成为 primary。
- **Classification definitions：** `P29-A` 为 physical/canonical 与 solver state/reference
  contract错误；`P29-B` 为 semantics正确但 exact objective gradient要求错误方向；
  `P29-C` 为 objective要求恢复但 generated dynamics/control authority 的符号或量级与冻结
  模型合同不一致；`P29-G` 为 converged solve恢复而 exact production lifecycle不恢复；
  `P29-F` 为 OCP bound 因果性改变动作；`P29-D` 为无 constraint/lifecycle解释时的
  cross-state/wheel trade-off；`P29-E` 为 future reference、terminal或有限域传播主导。
- **Ambiguity rule：** 多个机制同时成立、shadow solve改变多个因素、interaction不可辨识、
  solver分支不稳定或证据未满足定义时，结果必须为 `unresolved/REWORK`。T0 与 T1 可以有
  不同 primary cause，不能为追求共同解释而合并。
- **Terminal outcome：** Phase 29 PASS 只批准 root-cause attribution，不批准修复、调参、
  solver lifecycle变更或新增 WBC task。

## Open Questions / Decision Gates

- **DG29-00 / CLOSED PASS / CODEX+EVIDENCE — authority and lifecycle reproduction：** 固定 T0/T1
  target tick/state/reference、production update history与 artifact hashes；production-lifecycle
  replay的 requested wrench、status和可比 audit必须在预冻结 tolerance内复现 Phase 28
  authority。失败时先调查环境/日志/solver-state差异，不进入原因分类。
- **DG29-01 / CLOSED PASS / CODEX+EVIDENCE — state/reference contract：** physical perturbation、
  canonical 16-state error与每个 stage reference的 sign/frame/order/advance通过 golden
  vectors；任何 mismatch直接形成 `P29-A` candidate并须由双向 oracle确认。
- **DG29-02 / CLOSED PASS / CODEX+EVIDENCE — exact cost and dynamics derivatives：** 冻结有限域
  objective、perturbation尺度、active-set稳定性判据和 independent/generated derivative
  tolerances；只有 cost和model证据均闭合，才可进入 OCP trade-off 分类。
- **DG29-03 / CLOSED PASS / CODEX+EVIDENCE — converged oracle equivalence：** append-only SQP
  artifact除 convergence mechanism外与 production OCP逐字段/hash等价，并在无约束 golden
  corpus上与 repeated-RTI secondary oracle一致；不闭合则 oracle无效，不形成 `P29-G`。
- **DG29-04 / CLOSED PASS / CODEX+EVIDENCE — causal counterfactual method：** 在 primary结果不可见
  时冻结 state groups、single/pairwise矩阵、shadow relaxation、held-reference/terminal
  probes、翻转/主导/重复性门槛和ambiguity规则。
- **DG29-05 / CLOSED PASS / CODEX+EVIDENCE — unique root cause：** T0=`P29-E`、T1=`P29-D`，分别按冻结顺序只命中
  一个 P29-A～G primary cause，并由至少三种独立证据闭合；否则 REVIEW=REWORK。

## Interfaces and Compatibility

- 输入：Phase 28 T0/T1 authority snapshots与完整 case prefix；canonical 16-state measured/
  reference；current OCP config/generated artifact；左右 wheel-centre 12D requested wrench。
- 内部：production path保持
  `WheelState/Planner -> WheelAwareNmpcSolver -> Minimal WBC -> TorqueCommand`；新增路径仅为
  `captured OCP data -> offline diagnostic oracles -> report`，不得回写 live controller。
- 输出：production `TorqueCommand`、public state/command schema和normal log语义不变；
  Phase 29 additive evidence至少包含 solve-kind/lifecycle、stage `x/u/yref/cost`、bounds/
  multipliers/active set、objective/KKT、requested wrench、predicted acceleration、artifact/
  config/input hashes和classification reason。
- 必须保持：Phase 21～28 default behavior、fault/fail-zero/reset、deadline/update schedule、
  production artifact、normal formal threshold与已有 evidence不变。
- 允许改变：仅 namespaced offline oracle/config/script/test、append-only generated SQP
  diagnostic artifact、additive diagnostic extraction和 Phase 29 evidence。若可由 test-only
  accessor或现有 acados API读取，优先不扩展 public runtime interface。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| T01 | 关闭 authority、grounding 与影响面 | design source、Phase27/28 RECORD/manifest/raw evidence、live solver/model/controller/generator | hash manifest、exact T0/T1 ticks/snapshots、production/cold差异表、reuse/non-reuse map | CBM+coverage+source/Graphify核对；DG29-00输入完整；不修改生产文件 | done |
| T02 | 冻结 method、schema 与判据 | T01、Frozen Decisions | versioned perturbation/groups/interactions、solver tolerances、classification/ambiguity、non-overwrite schema | primary结果不可见时关闭DG29-02/04的数值门槛；schema synthetic validation | done |
| T03 | 建立 production-lifecycle与cold双重reproduction | T01/T02、T0/T1 case prefixes、current solver | 两类solve的逐tick request/status/audit corpus和target OCP data capture | production prefix复现authority；cold复现Phase28 oracle；混用/不一致显式报告 | done |
| T04 | 关闭 state/reference/error semantics | T03、advanceReference、16-state contract、orientation/wheel maps | T0/T1 physical-to-state golden vectors、全stage yref/error audit | ±theta/omega/x/v/xi/dxi、frame/order/sign/advance双向PASS；DG29-01 | done |
| T05 | 建立 exact cost/model/stagewise diagnostic oracle | T02～T04、generated OCP与C++ model | full-horizon objective decomposition、generated/independent Jacobians、stage x/u/yref/cost/bound/multiplier/KKT export | finite difference/analytic parity；objective重算一致；active-set变化不伪装为中心导数 | done |
| T06 | 建立并验证 converged full-solve oracle | T02/T03/T05、append-only SQP artifact | SQP manifest、equivalence diff、golden corpus、RTI-vs-converged report | DG29-03；同构项逐字段一致；收敛/stationarity/repeated-RTI cross-check PASS | done |
| T07 | 执行 T0 positive-feedback isolation | T03～T06、T0 authority snapshot | actual/single/pairwise counterfactual matrix、wrench/angular-acceleration/objective/KKT/stage report | 按分类顺序排除前层；candidate由interaction与独立证据闭合或标unresolved | done |
| T08 | 执行 T1 net-action isolation | T03～T06、T1 authority snapshot | actual/single/pairwise counterfactual matrix、common Fx/Ty、longitudinal acceleration/objective/KKT/stage report | restorative local derivative与non-restorative net action的抵消项可因果隔离或标unresolved | done |
| T09 | 隔离 constraint、cross-state 与 horizon 因果 | T05～T08、stage/active-set结果 | bound shadow、held-reference/terminal shadow、adjoint/KKT或等价stage contribution报告 | 仅执行仍需的后层；P29-F/D/E各满足冻结必要条件，shadow因素不混改 | done |
| T10 | 形成 T0/T1 classification 与 limited T2 holdout | T07～T09、DG29-05、T2少量snapshot | root-cause matrix、证据链、T2 left/right mechanism-consistency结果、limits | T0/T1各唯一P29-A～G且至少三类证据；T2不反向选择/调门槛 | done |
| T11 | 执行 replay/fault/regression 并组装审查输入 | T01～T10 authority | fresh replay、non-overwrite、invalid/superseded runs、production parity、regressions、REVIEW输入 | replay仅允许预声明非语义差异；normal control/fault/reset与Phase27/28基线一致 | done |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

以下命令和 evidence 入口是执行 Phase 29 时使用的冻结验证计划。

### Preflight / build

- 在任何 formal 输出目录创建前，以 `./.venv/bin/python` 探针实际脚本使用的 MuJoCo、
  NumPy、SciPy、CasADi/acados等依赖并记录解释器/版本；失败记为环境 gate，不记为模型或
  controller evidence FAIL。
- `./.venv/bin/python -m py_compile <phase29 scripts>`：全部 offline evaluator/generator
  wrapper先通过，再允许创建稳定输出根。
- 从 `ros_ws` 执行 Release build：
  `source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-up-to wheel_leg_mujoco --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DACADOS_ROOT=/home/t/opt/acados`。
- 从 `ros_ws` 执行：
  `source /opt/ros/jazzy/setup.bash && colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco && colcon test-result --verbose`。

### Component / oracle

- Authority：production-lifecycle replay在 target tick之前逐行对齐 state/reference、NMPC
  update/status/requested wrench与audit；cold snapshot单独对齐Phase28 perturbation结果。
- State/reference：T0 `±theta/±omega_y`、T1 `±x/±vx/±xi/±dxi` 的 physical sign、state
  ordering、frame和全stage reference/error由独立构造与production path双向一致。
- Objective：从导出的 `x/u/yref` 独立重算每个 stage与terminal cost，总和与solver audit在
  预冻结 tolerance内；state/input gradient用解析/automatic/finite-difference中至少两条链。
- Model sensitivity：generated discrete map、current C++/analytic Eq.(12)与central/one-sided
  perturbation在合法branch内一致；任何control-authority sign mismatch可定位到具体state/
  input/frame。
- Solver oracle：offline SQP满足预冻结 convergence/KKT gate；production、cold、converged
  三类状态不共享未声明iterate；重复运行和repeated-RTI cross-check可复现。
- Counterfactual：actual、single-removal、pairwise interaction均保存完整OCP输入和hash；
  非目标量、reference/bounds/lifecycle保持不变，除非该case明确标为shadow probe。
- Constraint/horizon：bound relaxation每次只改变一个预声明constraint group；held-reference/
  terminal probe单独运行，禁止在同一solve同时放宽bound和改变reference/terminal。
- Evaluator：synthetic P29-A～G、nonsmooth、multiple-cause、missing/non-finite和unresolved
  fixtures覆盖决策树；ambiguity必须稳定输出REWORK。

### Formal / evidence

- T0/T1分别保存 authoritative production prefix、target snapshot、cold/production/converged
  solution、完整stage trajectory、cost/KKT/active set、requested wrench与predicted acceleration。
- 分类先检查 P29-A/B/C；其后用 production-vs-converged 判 P29-G；converged仍非恢复时才
  执行 P29-F/D/E 的因果shadow tests。
- `P29-B` 不能只凭单个 weight或单stage gradient；`P29-C` 不能只凭WBC/MuJoCo response；
  `P29-F` 不能只凭active flag；`P29-D/E` 不能只凭一次sign flip。
- T0/T1各至少由 frozen perturbation、counterfactual solve、predicted action/acceleration、
  stage/horizon、full-solve中三类相互独立证据闭合；若证据共享同一未验证假设，不计作三类。
- T2 holdout只在T1 root cause和所有threshold冻结后运行，只输出same/not-same/unresolved；
  T3不运行。
- 每个run保存命令、interpreter/dependencies、git/worktree context、model/config/controller/
  solver/artifact/schema/input hash、termination reason、raw data、summary和source links；重跑写
  新目录，失败与inconclusive不覆盖。

## Acceptance Criteria

- [x] T01～T11完成，DG29-00～05由真实 evidence关闭；所有invalid/inconclusive/superseded
  run追加保留。
- [x] T0/T1 production-lifecycle prefix与Phase28 authority逐tick复现；cold frozen solve和
  converged solve的不同语义被明确记录，未互相替代。
- [x] state/reference/error、exact finite-horizon objective与generated model/control
  sensitivity通过独立oracle，任何不连续active-set被正确标为nonsmooth。
- [x] append-only offline SQP与production OCP除收敛机制外逐项同构，满足预冻结KKT/
  convergence gate且不改变production artifact或runtime path。
- [x] T0、T1分别按冻结顺序唯一归入一个P29-A～G，并由至少三种独立证据闭合；无法排除
  interaction、branch或多因子时REVIEW=REWORK。
- [x] constraint、cross-state或horizon归因均有单因素diagnostic counterfactual因果证据，
  不从active flag、weight大小或单组sign flip直接推断。
- [x] T2只形成T1已关闭机制的left/right holdout一致性结论；T3和修复方案保持范围外。
- [x] diagnostics/offline artifacts不反馈production；default control、fault/fail-zero/reset、
  schedule、public interfaces、Phase27/28 threshold/evidence和历史回归保持一致。
- [x] REVIEW无blocking finding；只有REVIEW=PASS后才创建RECORD并将ROADMAP改为complete。

## Execution Notes

T01～T11 已按 gate 顺序执行。冻结 method 为
[`phase29_nmpc_root_cause_v1.json`](../../../../simulation/mujoco/config/phase29_nmpc_root_cause_v1.json)，
最终 offline oracle 为 append-only `phase29_wheel_aware_nmpc_sqp_v2`；production artifact、
solver wrapper、cost、bounds、reference、timing和WBC均未修改。formal-v1 与 fresh replay-v4
的五个语义文件逐字节一致，summary SHA-256 均为
`a86573b537bbbd783c1ff8c78ff7f64d41b531b2af0e2bbd7256870c5f9c2172`。

完整数值链见 [`evidence/root-cause-audit.md`](evidence/root-cause-audit.md)。T0 唯一关闭为
`P29-E_horizon_reference_propagation`：production/repeated-RTI/converged均非恢复，只有移除
terminal objective或其base-longitudinal项才恢复。T1唯一关闭为
`P29-D_cross_state_coupling`：converged、terminal/reference与bound shadows均不恢复，
attitude group removal恢复且pairwise证据确认其主导，wheel-rate为次级interaction。T2 right
与T1一致，left原动作已恢复，故为`not_same`。

验证实际结果：Python依赖探针与`py_compile` PASS；Release colcon build PASS；三个ROS package
为`33 tests, 0 errors, 0 failures, 0 skipped`；Phase28方向oracle原值PASS；向formal-v1重跑被
non-overwrite gate拒绝。replay-v2因缺少显式关系元数据被v4 supersede；v3为缺少acados动态库
路径的环境gate失败且无语义输出；这些目录和pre-formal SQP v1 artifact均追加保留。

任何需要改变 production model/reference/cost/bounds/solver lifecycle、WBC、timing或安全
语义的发现都必须停止当前归因、REWORK 本 PLAN，并把实际修复放入后续 Phase；不得在
Phase 29 内顺带修复后继续宣称原根因已独立关闭。

## Blockers

None.
