# Phase 30: NMPC Corrective-Action Formulation Repair — PLAN

Status: `review`

Supersedes for execution: the v1 alpha/beta-only route recorded below and preserved in
[`REVIEW-v1-2026-08-29-REWORK.md`](REVIEW-v1-2026-08-29-REWORK.md). The v1 evidence remains
authority for rejecting terminal-x-only and running-attitude-only scalar tuning; it is not overwritten.

Design source: [NMPC Corrective-Action Formulation Repair](../../../mujoco/NMPC%20Corrective-Action%20Formulation%20Repair.md).
本 PLAN 接受设计稿的 two-branch、one-cause/one-intervention 和 local-before-closed-loop
原则，但修正 T1 因果语义、running/terminal 权重耦合、candidate sweep 与鲁棒裕量。
设计稿与本 PLAN 冲突时，以本 PLAN 为准。

## Goal

在 Phase 27～29 的 current-nominal plant、16-state model、internal-wrench contract、WBC、
solver family、timing和安全语义不变的前提下，只用两个独立标量 formulation change：

- T0：terminal base-longitudinal absolute-position weight scale `alpha`；
- T1：running attitude-group weight scale `beta`；

先恢复 frozen snapshot 的鲁棒 corrective direction，再分别通过 isolated closed-loop，最后
组合并验证 T0/T1 与 limited T2/T3 regression。若任一 scalar repair hypothesis 不能通过
预冻结 gate，则 Phase 30 为 REWORK；不在本 Phase 升级为 cost-structure redesign、wheel-rate
production retuning或 WBC task add-back。

## Design Audit

方案总体方向合适，但两个scalar candidate都必须先被当作待证的最小repair hypothesis。
执行前必须接受以下修正：

- **T0 证据边界：** Phase 29 `terminal_without_base_longitudinal` 同时清零 indices `[0,6]`
  （terminal x与vx），没有单独证明`Qe,x=0`足够。`alpha`仍是合理的最小第一候选，但必须先
  通过x-only direct-weight screen；失败即R30-A，不把group shadow误写成single-weight批准。

- **T1 证据边界：** Phase 29 `single_removal.attitude` 的 live evaluator 是把 state indices
  `[3,4,5,9,10,11]` 置为 reference，并非把 attitude cost weight 置零。它证明 attitude-error
  interaction 是主导 coupling，但没有直接批准 `Q_att` scaling。故 `beta` 是 Phase 30 的
  repair hypothesis，必须先用真实 running-weight counterfactual 关闭因果 gate。
- **running/terminal 解耦：** current generator 使用 `Qe=10*Q`。若直接修改 `state_weight`，
  running 与 terminal attitude 会一起变化，违反设计稿“T1 不改 terminal formulation”。
  Phase 30 必须显式建立独立的 16D running/terminal scale；默认全 1 时逐元素复现 Phase 27。
- **有限搜索：** `alpha/beta` grid 现在冻结为
  `{0, 0.125, 0.25, 0.5, 0.75, 1}`，禁止看结果后加点、二分或连续优化。`beta=0` 仅为因果
  screen；如果只有 `beta=0` 通过，T1 scalar repair 判 `R30-B`，不得作为 production candidate。
- **不能刚好过零：** candidate 不只要求符号翻转，还须达到相对 zero-scale screen 的 10%
  corrective margin和绝对 floor；最大通过 scale 才能入 isolated closed-loop。closed-loop失败
  后不得回到 grid 选择“跑得更好”的次优点。
- **范围收缩：** 设计稿中的 terminal/running responsibility redesign、structured cross-state
  redesign与 wheel-rate production scaling均移出本 Phase。wheel-rate只允许 diagnostic-only
  复核；任何 production `gamma` 都需新 Phase。
- **T2/T3 语义：** T2 right只验证T1机制是否消失，T2 left只做无新回归；不宣称完整turning
  获批。T3不要求把Phase27既有native-stationarity失败修好，但禁止出现更早或新的失败层。

## Current State and Grounding

- [Phase 29 RECORD](../29-nmpc-corrective-root-cause-audit/RECORD.md) 冻结 T0=`P29-E`
  terminal base-longitudinal propagation、T1=`P29-D` attitude-dominant cross-state coupling；
  production/cold/repeated-RTI/converged语义已经分离。
- Current OCP config 为
  `simulation/mujoco/config/phase27_acados_ocp_v2.json`：16 states、12 inputs、`Ts=20 ms`、
  `N=20`、two 10 ms RK4、`state_weight`和单一`terminal_weight_multiplier=10`。
- Current generator 在 `tools/experiments/generate_phase27_acados_solver.py` 中构造
  `W=diag(Q,R)`、`W_e=diag(10Q)`；current C++ audit同样使用一套`kStateCost`和标量
  `kTerminalMultiplier`。因此 T0/T1 独立修改需要同时修正 generated OCP 与 C++ objective/
  costate audit，不能只改 JSON。
- `WheelAwareNmpcSolver` 由 `ControllerCore` 的 Phase27 mode直接持有；public
  `RobotState/TorqueCommand` 和 wheel-centre interaction-wrench boundary无需变化。
- CBM project `W_L_ws` generation `2026-08-29T06:47:42Z`。solver/controller/config候选无
  recorded gap；CMake有当前worktree变化并已直接读取。`tools/`、`docs/`按策略不入CBM，
  generator/evaluator/Phase evidence均直接读取。Graphify只用于历史OCP/Phase关系。

## Scope

- 冻结 Phase 29 authority snapshots、prefix、method、model/config/artifact hashes与阈值；建立
  namespaced Phase 30 method/config/schema和append-only evidence。
- 为 offline diagnostic 增加独立 `running_state_weight_scale[16]` 与
  `terminal_state_weight_scale[16]`，默认全1必须复现 Phase29 v1 semantic authority。
- T0只令terminal index `0`为`alpha`；running weights及其他terminal indices保持1。
- T1只令running indices `[3,4,5,9,10,11]`为`beta`；terminal scale全部保持1。
- 先以converged SQP作primary、production-lifecycle RTI replay作secondary，完成zero-scale
  screen、固定grid和active-set/KKT审计。
- 在结果不可见前冻结candidate选择规则；每支只允许一个selected candidate进入isolated
  closed-loop，不按轨迹表现回选。
- isolated T0/T1均PASS后才组合`alpha+beta`；重新验证两份authority snapshot和完整闭环。
- 最终candidate生成一个append-only static SQP-RTI artifact；其model、horizon、solver、
  bounds、reference、input weights与Phase27逐字段相同，差异仅为批准的running/terminal
  state cost entries和namespaced model symbol。
- 更新C++ objective/costate audit为独立running/terminal数组，并以generated JSON和独立
  recomputation逐项验证；production artifact切换只在所有offline/isolated gate关闭后进行。
- 运行T0/T1、limited T2 left/right、T3 non-regression、fault/reset/deadline/replay和历史回归。

## Out of Scope

- 修改plant、Eq.(12)、state/input ordering、frame/sign、physical parameters、wheel-centre
  wrench semantics、reference advance、horizon、Ts、RK4、SQP-RTI/HPIPM或warm-start lifecycle。
- 修改input weights、bounds、state envelope、planner、WBC 42D/104-row topology、ProxQP、
  task set、contact/torque/workspace limits、2/10/20 ms schedule或fault priority。
- terminal group restructuring、terminal/running responsibility redesign、non-diagonal/cross-term
  cost、wheel-rate production scaling、提高Qx/Qv或任何第三个调参维度。
- 为closed-loop PASS反复扩充grid、降低margin、放宽Phase27/29 threshold或选择未预声明case。
- 新增pitch/base-X/rolling/height/leg WBC task、fast inner loop或bandwidth architecture。
- turning-specific修复、identified/CAD profile、真机、Hardware Adapter、STM32/树莓派结论。

## Frozen Decisions

- **Authority/non-overwrite：** Phase29 formal-v1 + replay-v4为归因authority；Phase30所有
  config、artifact和run使用新namespace与空输出目录，保存`source_run/replay_of/supersedes`。
  invalid、environment-fail、inconclusive和superseded run全部追加保留。
- **Exact cost parameterization：** 令baseline normalized state cost为`q`，则
  `Q_run=diag(q .* s_run)`，`Qe=diag(10*q .* s_terminal)`。baseline profile两scale全1；
  T0 profile仅`s_terminal[0]=alpha`；T1 profile仅
  `s_run[[3,4,5,9,10,11]]=beta`；combined同时应用两者。input cost不变。
- **Grid：** `alpha_grid=beta_grid=[0,0.125,0.25,0.5,0.75,1]`。每个值固定相同
  lifecycle、seed/reset、reference、bounds、perturbation和solver tolerances；禁止插值补点。
- **Zero-scale causal screens：** T0 `alpha=0,beta=1`必须直接证明x-only terminal repair并
  与Phase29 `[x,vx]` group shadow方向一致，通过T0 local gate；T1 `alpha=1,beta=0`必须直接证明running-attitude weight removal令T1
  longitudinal net action恢复且attitude自身不失稳。任一失败即R30-A/B，不进入对应sweep。
- **Corrective metrics：** 对误差`e`与同轴预测加速度`a`定义`C=-e*a`，`C>0`为恢复；
  对matching local derivative定义`D=-d(a)/d(e)`，`D>0`为恢复。central derivative只在两侧
  status、active set与solution branch一致时使用，否则one-sided并判candidate不满足鲁棒gate。
- **Margin：** 每个selected candidate的所有目标`C/D`必须大于
  `max(absolute_floor, 0.10*zero_scale_metric)`。`C` absolute floor=`1e-6`；linear/angular
  acceleration derivative floor=`1e-3 s^-2`。zero-scale metric本身也必须严格高于floor。
- **T0 samples：** authority action/snapshot、Phase29原`theta±0.002 rad`与
  `omega_y±0.01 rad/s`，再加相邻前一/后一NMPC update problem；要求pitch angle/rate的
  `C`与matching`D`全部恢复，x/vx不产生反向net action，chart/bounds/active-set gate通过。
- **T1 samples：** authority action/snapshot和相邻前一/后一NMPC update problem；要求
  `e_x<0,e_v<0`时`a_x>0`且`C_x/C_v`过margin，并对`r_x/r_y/r_z±0.002 rad`及
  `omega_x/omega_y/omega_z±0.01 rad/s`检查matching angular corrective `C/D`过margin。
- **Selection：** `alpha_star`和`beta_star`各取通过全部local/KKT gate的最大grid值；
  `beta_star=0`不允许。每支只运行这个selected candidate的isolated closed-loop；失败后不
  回选更小值。无candidate则R30-A/B。
- **Primary solver：** sweep和counterfactual使用converged SQP；production-lifecycle replay
  必须保持真实prefix iterate history并对selected candidate同向。二者不一致为unresolved/
  REWORK，不能归咎RTI后继续集成。
- **Runtime-vs-static parity：** grid使用现有acados runtime `W/W_e` setter，避免每个grid
  codegen。selected combined candidate生成唯一append-only static artifact；其next/A/B和
  solve output须与同权重runtime oracle在冻结corpus内一致。
- **Isolated-before-combined：** T0-only和T1-only分别完成local与closed-loop gate后才组合。
  combined必须重新通过两支local margin，不能用组合补偿某支isolated失败。
- **Wheel-rate boundary：** 可复跑`gamma=0` diagnostic确认secondary interaction，但不得把
  wheel-rate weight变化带入selected/combined/production candidate。若beta恢复方向但margin
  不足，结论仍是R30-B并另立Phase。
- **Closed-loop no-retune：** 每支selected scale在首次authority closed-loop前冻结。closed-loop
  FAIL只进入R30-C/E或REWORK，不根据轨迹返回sweep选另一个值。
- **Terminal outcome：** PASS只批准这两个scalar formulation changes用于current-nominal
  simulation production profile；不批准identified/real、turning architecture或新inner loop。

## Open Questions / Decision Gates

- **DG30-00 / CLOSED / CODEX+EVIDENCE — baseline and method authority：** hashes、snapshots、
  prefix lifecycle、state groups、grid、margin与schema完整；baseline scale=1逐字节/数值复现
  Phase29 authority且normal Phase27 behavior在声明差异外一致。
- **DG30-01 / CLOSED / CODEX+EVIDENCE — cost decomposition parity：** independent running/
  terminal scale在generator、runtime setter、C++ objective/costate与independent recomputation
  中逐元素一致；legacy all-one profile无语义变化。
- **DG30-02 / REWORK / CODEX+EVIDENCE — T0 scalar causality：** alpha=0 screen与固定grid通过
  T0 samples、margin、KKT/active-set；唯一选择alpha_star或输出R30-A。
- **DG30-03 / REWORK / CODEX+EVIDENCE — T1 scalar causality：** beta=0 direct cost screen确认
  Phase29 attitude-error finding可由running attitude scaling修复；固定grid唯一选择非零
  beta_star或输出R30-B。
- **DG30-04 / BLOCKED / CODEX+EVIDENCE — isolated closed loops：** selected T0-only/T1-only各自
  消除原首失效，不出现同机制延迟或新安全/resource/feasibility失败；否则R30-C/E。
- **DG30-05 / BLOCKED / CODEX+EVIDENCE — combined/static production parity：** combined local
  gates不互相破坏，static artifact与runtime oracle同构同解，production wrapper audit一致。
- **DG30-06 / BLOCKED / CODEX+EVIDENCE — closed-loop and regression release：** T0/T1 combined
  formal、T2 limited validation、T3 non-regression、deadline/fault/replay/history全部满足冻结规则。

## Interfaces and Compatibility

- 输入：Phase29 T0/T1 authority prefixes/snapshots/raw stage evidence、Phase27 OCP config/model/
  generated artifact、current ControllerCore/WheelAwareNmpcSolver/Minimal WBC和formal matrix。
- 内部新增：Phase30 method/config；offline weight setter；selected static generated artifact；
  independent `running_state_cost[16]`/`terminal_state_cost[16]` audit constants。
- public `RobotState`、`TorqueCommand`、`WheelAwareNmpcProblem/Result`、interaction-wrench、
  reference/state order和normal logging schema保持兼容；新增诊断列只能append且默认关闭。
- 有意变化：Phase27 minimal-NMPC production profile的批准cost entries及其T0/T1动作/轨迹。
  Phase27/28原失败轨迹不能要求bitwise不变；其余mode、fault/reset和未涉及contract必须回归。

## Tasks

| ID | Task | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- |
| T01 | 冻结authority、grounding、hash与影响面 | Phase29→30 handoff、live cost flow、reuse/non-reuse map | CBM+coverage+source/Graphify；DG30-00输入完整 | done |
| T02 | 冻结repair method/schema | exact scales/grid/margins/samples/lifecycle/non-overwrite config | synthetic schema/evaluator fixtures；结果不可见前冻结 | done |
| T03 | 建立running/terminal独立cost oracle | runtime W/W_e setter、objective/KKT recomputation、all-one baseline | generated/runtime/independent parity；DG30-01 | done |
| T04 | 执行T0 zero-screen与alpha sweep | T0 causal matrix、alpha_star或R30-A | converged+production lifecycle、local C/D、active-set/KKT | done |
| T05 | 执行T1 zero-screen与beta sweep | T1 direct-weight causal matrix、beta_star或R30-B | running-only/terminal-unchanged；六轴attitude和longitudinal margin | done |
| T06 | 运行T0-only isolated closed loop | selected T0 report与raw logs | 原0.58s机制消失、无延迟同机制、x/vx/WBC/plant healthy | blocked |
| T07 | 运行T1-only isolated closed loop | selected T1 report与raw logs | 原0.45s机制消失、attitude/wheel/WBC/plant healthy | blocked |
| T08 | 组合candidate并复核local causality | combined snapshot/prefix/cross-branch matrix | 两支margin保持；不得用组合挽救isolated FAIL | blocked |
| T09 | 生成并接入唯一static production artifact | append-only artifact/manifest、C++ audit、parity corpus | runtime-vs-static next/A/B/u0/KKT parity；DG30-05 | blocked |
| T10 | 运行combined T0/T1 formal | complete closed-loop evidence与failure classification | Phase27 safety、resource、stationarity、contact/torque gates | blocked |
| T11 | 运行T2/T3、fault/deadline/history regression | limited turning、T3 non-regression、33+ tests、replay | DG30-06；未声明变化为零；non-overwrite | blocked |
| T12 | 组装repair-causality matrix与审查输入 | evidence index、invalid/superseded map、REVIEW input | T01～T11/DG30-00～06闭合，否则REWORK | done |

任务状态只使用`todo / doing / done / blocked`。

## Failure Classification

- `R30-A_terminal_scalar_insufficient`：alpha=0 screen或alpha grid不能鲁棒修复T0；后续Phase才
  能讨论terminal structure redesign。
- `R30-B_attitude_scalar_insufficient`：beta=0不能直接修复，或无非零beta通过；后续Phase才
  能讨论structured cross-state/wheel-rate redesign。
- `R30-C_local_repaired_closed_loop_unstable`：local因果链PASS但isolated/combined闭环仍失稳；
  后续Phase才可重新审计bandwidth/fast stabilization。
- `R30-D_turning_new_mechanism`：T0/T1 repaired但T2出现新机制；另立turning Phase。
- `R30-E_feasibility_stationarity_resource_regression`：repair破坏OCP/WBC/resource gate；停止
  降权，不通过继续调参处理。
- 多类同时成立或证据不足：`unresolved/REWORK`，不得挑选最方便的原因。

## Validation Plan

### Preflight / component

- stable output创建前以`./.venv/bin/python`探针MuJoCo/NumPy/SciPy/CasADi/acados并记录版本，
  随后`py_compile`全部Phase30 generator/oracle/evaluator；环境失败不记control evidence FAIL。
- all-one cost profile复现Phase29 T0/T1 production/cold/converged request、objective、stage
  trajectory和classification；任何差异先关闭DG30-01。
- 对每个grid case保存完整W/W_e、x/u/yref、objective decomposition、residual/KKT、bounds/
  multipliers、active set、u0、predicted acceleration与input/config/artifact hashes。
- selected static artifact与Phase27逐字段diff；允许差异白名单仅model symbol和批准cost entries。
- component corpus覆盖equilibrium、positive/negative perturbation、T0/T1相邻updates、cold reset、
  repeated RTI、converged SQP和invalid/nonfinite input。

### Closed loop / formal

- isolated T0/T1与combined使用Phase27 frozen case定义、duration、安全阈值和resource gate；
  diagnostic continuation至少覆盖原first-failure之后同等窗口，且完整formal覆盖原case duration。
- T0记录pitch/rate/x/vx、Fx/Ty、requested/realized wrench、model/MuJoCo acceleration、WBC
  residual/slack、stationarity、contact/torque margin；不得只看failure tick消失。
- T1记录x/vx及六轴attitude、common/differential wheel state/wrench、reference following、同一
  solver/WBC/plant指标；不得只看net acceleration单点。
- T2 right要求不再复现T1 attitude-dominant mechanism；T2 left若出现新首失效则R30-D，不为
  兼容它修改alpha/beta。T3保持baseline failure layer/time不提前且无新层，不要求转PASS。
- fault/fail-zero/latch/reset、2/10/20 ms schedule和production deadline使用Phase27门槛；
  Release build与`colcon test`从`ros_ws`执行。
- fresh replay写新目录；语义结果仅允许manifest timestamp/output-root等预声明差异；向已有
  目录重跑必须在写入前失败。

## Acceptance Criteria

- [ ] T01～T12完成，DG30-00～06由真实evidence关闭；invalid/inconclusive/superseded追加保留。
- [ ] T0 live-evidence边界被正确处理：alpha通过x-only terminal-weight causal screen，而非把
  Phase29 `[x,vx]` group shadow误称为single-weight evidence。
- [ ] T1 live-evidence边界被正确处理：beta通过直接running-weight causal screen，而非把
  Phase29 attitude-state removal误称为weight-removal evidence。
- [ ] running/terminal cost独立，all-one baseline复现；除alpha/beta外所有OCP字段不变。
- [ ] alpha_star与非零beta_star由固定grid和预冻结margin唯一选择，无post-hoc补点或回选。
- [ ] T0-only、T1-only isolated local/closed-loop均PASS，combined再次保持两支因果方向。
- [ ] final static artifact与runtime oracle parity PASS，C++ objective/costate audit使用准确的
  running/terminal权重且production deadline满足Phase27门槛。
- [ ] combined T0/T1 formal消除原机制且不引入新safety/WBC/plant/resource failure。
- [ ] T2 right机制消失、T2 left无未归因blocking regression；T3无更早/新失败，不扩大结论。
- [ ] fault/reset/exact-zero、replay、non-overwrite、历史mode与仓库测试PASS，阈值未放宽。
- [ ] REVIEW无blocking finding；仅PASS后创建RECORD并把ROADMAP设为complete。

## Execution Notes

2026-08-29执行direct-weight causal gate。以`./.venv/bin/python`完成依赖探针和`py_compile`，
用现有Phase29 SQP/RTI artifact的runtime `W/W_e` setter运行冻结六点grid。all-one profile对
Phase29 T0/T1 production/converged `u0`与converged objective均为exact parity，prefix最大误差
分别为`7.77e-16/7.22e-16`。formal-v1与fresh replay-v2的两个语义文件逐字节一致。

T0 `alpha=0`在三个update的pitch当前点`C`为正，但matching derivative均反恢复
（`D_theta=-71.37..-70.74`、`D_omega=-14.70..-14.57`），且x/vx guard均反恢复；固定grid无
candidate，结论`R30-A_terminal_scalar_insufficient`。T1 `beta=0`在authority tick 44的x/vx
`C=-6.93e-5/-3.16e-4`，没有复现Phase29 attitude-state-removal的恢复方向，并且pitch
`D_theta=-112.39`、`D_omega=-16.21`；固定grid无非零candidate，结论
`R30-B_attitude_scalar_insufficient`。按冻结gate停止T06～T11，未生成candidate、未修改或接入
production solver/weights/C++ audit，也未运行不再具有放行意义的closed-loop与仓库回归。
证据见[`evidence/repair-causality.md`](evidence/repair-causality.md)及append-only formal目录。

## Blockers

- DG30-02：`R30-A_terminal_scalar_insufficient`。
- DG30-03：`R30-B_attitude_scalar_insufficient`。
- 两项pre-integration gate均失败，T06～T11按PLAN禁止继续；需要新的formulation-design Phase，
  不能在本Phase追加第三个调参维度或WBC task。

## V2 Structured Formulation Route — 2026-08-29

用户批准在同一 Phase 内重划路线，不另开 Phase。设计输入为 pasted
“Structured NMPC Cost / Terminal Formulation Redesign”。方向获批，但作以下必要收缩：

- A1（terminal x/vx均移除）与原稿 A3 在当前 diagonal 16-state terminal cost 下数学相同，
  只保留 A1；A3 不作为虚假独立 candidate。
- A2 冻结为 terminal x=0、terminal vx=`1*q_vx`，即从 legacy `10*q_vx` 改为与单个
  running-stage normalized weight相同；不是 sweep。
- B1 只是 baseline diagonal Q 的分组记账，不是数学 candidate；只作为 contribution audit。
- B3 error-dependent nonlinear shaping 暂不进入本路线，避免引入未冻结 heuristic。
- B2 只使用两个 PSD-by-construction correlation candidates：在 normalized running Q 上令
  `Q[0,4]=Q[4,0]=rho*sqrt(q_x*q_ry)`、
  `Q[6,10]=Q[10,6]=rho*sqrt(q_vx*q_omega_y)`，`rho=+0.25/-0.25`。
- B4 只移除 wheel-rate common-mode running responsibility并保留 baseline differential
  responsibility：indices 14/15 block为`q/2*[[1,-1],[-1,1]]`，terminal保持baseline。
- T0 candidate priority固定`A1 → A2`；两者都过 gate时选更简单且有Phase29直接支持的A1。
  T1 priority固定`B4 → B2_pos → B2_neg`；多个PASS时按此结构简洁/因果直接顺序选择，
  不按closed-loop误差挑选。

### V2 Frozen Local Gates

- Baseline必须精确复现Phase30 v1/Phase29 authority。
- T0在ticks 54/56/58要求`D_ry>1e-3`、`D_omega_y>1e-3`且`C_x,C_vx>=-1e-6`；
  converged SQP KKT/active branch PASS，production RTI在authority action同向。
- T1在ticks 42/44/46要求authority error为负时`a_x>0`，并要求六轴
  `D_rx,D_ry,D_rz,D_omega_x,D_omega_y,D_omega_z>1e-3`；同一KKT/branch与RTI parity gate。
- 所有 Q/Qe 对称；最小特征值`>=-1e-10`。对正特征子空间报告condition number；任何
  solver feasibility/stationarity regression淘汰为R31-E。
- candidate set在本节写入后冻结；primary结果后不加点、不改rho、不再定义新block。
- 若任一分支无candidate，输出R31-A/B并再次REWORK；不得进入closed-loop或production。
- 两支local均PASS才各运行一个isolated closed loop；isolated均PASS才组合并生成唯一static
  artifact，之后才运行T2/T3/fault/deadline/history regression。

### V2 Tasks and Gates

| ID | Task | Gate | Status |
| --- | --- | --- | --- |
| V2-T01 | 冻结structured candidate spec、hash与v1 handoff | DG30V2-00 baseline/non-overwrite | done |
| V2-T02 | 建立full-matrix runtime W/W_e oracle与PSD/KKT audit | DG30V2-01 exact cost parity | done |
| V2-T03 | 执行A1/A2 T0 causal matrix | DG30V2-02 select T0或R31-A | done |
| V2-T04 | 执行B2±/B4 T1 causal matrix | DG30V2-03 select T1或R31-B/E | done |
| V2-T05 | selected T0/T1 isolated closed loop | DG30V2-04 both isolated PASS | blocked |
| V2-T06 | combined local/closed-loop与static artifact | DG30V2-05 parity and formal PASS | blocked |
| V2-T07 | T2/T3/fault/deadline/history/replay | DG30V2-06 release evidence | blocked |
| V2-T08 | REVIEW/RECORD/ROADMAP | 仅全部PASS complete，否则REWORK | done |

V2 execution 保留上方 v1 tasks/gates的历史状态；V2 状态以本表为当前authority。

### V2 Execution Result

2026-08-29：full-matrix all-one profile对Phase29 T0/T1 production/converged `u0`及objective
均exact parity。A1/A2均PSD且production RTI pitch score恢复为`-0.2951/-0.2801`，但三个
update的converged `D_ry`仍约`-47.97..-50.06`、`D_omega_y`约`-12.44..-12.78`，x/vx
guards均为负，故R31-A。B2±/B4均PSD、KKT与cost recomputation通过，但authority production
score仍为`+0.001047/+0.001226/+0.001028`；ticks 44/46的`a_x`仍为负，pitch derivatives
仍约`D_ry=-113.77..-116.29`、`D_omega_y=-17.53..-17.71`，故R31-B。

formal-v1与fresh replay-v2的`structured_causal_matrix.json`和`summary.json`逐字节一致；
existing-output non-overwrite在solver构造前拒绝。按gate未运行V2-T05～T07，未生成static
artifact、未修改production C++/solver/WBC。详见
[`evidence/structured-formulation-v2.md`](evidence/structured-formulation-v2.md)。

## V3 Reference-Consistency Route — 2026-08-29

用户批准依据 pasted “Reference-Consistent NMPC Formulation Audit” 在同一 Phase 内重修路线，
不另建 Phase。v1/v2 的 cost-candidate FAIL 继续保留，但不再把它们解释为需要继续扩展 Q/Qe
候选；v3 首先审计 production reference 与冻结 16-state discrete model 是否一致。

### V3 Frozen Decisions

- Gate 0 必须从 Phase 28 raw control log 重建 Phase 29 的 T0/T1 problem prefix，并精确复现
  action tick、state/reference/rotation/logged wrench和production request。失败时停止，不能把
  reference audit 的差异解释为控制结论。
- Gate 1 对 T0/T1 authority update及相邻前/后 NMPC update，导出完整 20-stage `x_ref[k]`。
  current input reference固定为 production 的 12D equilibrium input；discrete map直接复用
  Phase 27 generator 的 `disc_dyn_expr`，不得以连续 flow 或简化 Euler 近似代替。
- stage defect定义为 `d=x_ref[k+1]-f_d(x_ref[k],u_ref[k],R_ref)`；以Phase27
  `state_error_scale`归一化，同时保存原始16D、base position、attitude、linear/angular
  velocity以及wheel common/differential position/rate分组结果。
- Gate 2 对每个stage独立求解有界12D best input：最小化`||d/scale||_2`，input bounds保持
  Phase27原值。使用SciPy trust-region least-squares，从equilibrium input启动，`xtol/ftol/
  gtol=1e-12`、`max_nfev=500`；不加入input regularization或cost权重。
- 结果不可见前冻结判定：令`E=max(abs(d/scale))`。Case A要求
  `E_current>=1e-2`、`E_best<=1e-3`且`E_best/E_current<=0.1`；Case C要求
  `E_current<=1e-3`；Case B要求`E_current>=1e-2`且`E_best>1e-3`。其余为
  `unresolved_transition_band`，不得事后移动阈值。
- Gate 2若A，进入stage-varying 12D feedforward/reference construction；若B，才运行R0/R1/R2/R3
  source attribution；若C或transition band，转入model-adequacy branch。任何分支都先做offline
  local RTI/SQP复核，再决定是否允许closed-loop。
- v3禁止调Q/Qe、terminal multiplier、horizon、solver、bounds、WBC或production lifecycle。
  append-only evidence需保存method/source/model hashes、optimizer termination、active bounds与
  fresh replay；formal/replay语义文件必须逐字节相同。

### V3 Tasks and Gates

| ID | Task | Gate | Status |
| --- | --- | --- | --- |
| V3-T01 | 归档v2并冻结reference-consistency method/hash | DG30V3-00 route/non-overwrite | done |
| V3-T02 | 重建T0/T1 authority与production prefix | DG30V3-01 exact baseline authority | done |
| V3-T03 | 导出current full-horizon defect及分组 | DG30V3-02 exact discrete-model Gate 1 | done |
| V3-T04 | 求解bounded best-input defect并分类A/B/C | DG30V3-03 pre-frozen causal classification | done |
| V3-T05 | 执行A feedforward、B source attribution或M model branch | DG30V3-04 conditional evidence | done |
| V3-T06 | 对唯一证明成立的formulation做local RTI/SQP复核 | DG30V3-05 corrective direction/KKT | blocked |
| V3-T07 | 仅在local PASS后运行closed-loop及release regression | DG30V3-06 integration authority | blocked |
| V3-T08 | REVIEW/RECORD/ROADMAP | 仅全部PASS complete，否则REWORK | done |

V3表为当前执行authority；上方v1/v2表仅保留历史。

### V3 Execution Result

2026-08-29：Gate 0从Phase28 raw log精确重建T0/T1。state/reference/rotation/logged wrench
误差均为0；production prefix最大request误差分别为`7.77e-16/7.22e-16`，production与
converged authority `u0`误差均为0。

Gate 1/2对ticks T0=`54/56/58`、T1=`42/44/46`及20个stage使用Phase27原始
`disc_dyn_expr`。六个problem的current最大归一化defect均为`8.7378e-4`，低于预冻结
small gate `1e-3`，故一致分类为`P31-C_current_reference_already_consistent`；bounded
best-input为`6.6500e-5`，但不能据此批准feedforward变更。

按Case C进入Branch M。以每个authority邻域的真实MuJoCo state为初值，固定initial yaw
anchor，分别用记录的requested wrench与两次10 ms realized-wrench均值作open-loop predictor。
两条预测几乎相同，但realized-input模型在20 ms的最大归一化误差为`0.1296..0.2258`，超过
预冻结`0.1`；主导量为单轮`dxi_left/right`，绝对误差`0.0194..0.0340 m/s`。同期base
angular-velocity分组最大仅约`0.0391` normalized，故结论收缩为
`P31-F_wheel_state_model_adequacy_failure`，不是整个base rigid-body模型均失败。

100 ms误差继续扩大至`0.5627..1.0220`。200/400 ms被原Phase28 diagnostic safety latch
截断，不将截断后的`dt=0`样本混入evidence；20 ms local gate已经失败，因此不得进入cost、
feedforward、RTI/SQP corrective或closed-loop candidate。formal-v1与fresh replay-v2的四个
语义文件逐字节一致。详见
[`evidence/reference-consistency-v3.md`](evidence/reference-consistency-v3.md)。
