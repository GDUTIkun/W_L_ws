# Phase 28: Minimal closed-loop drift / divergence attribution — PLAN

Status: `complete`

Design source: [Minimal Closed-Loop Drift / Divergence Attribution](../../../mujoco/Minimal%20Closed-Loop%20Drift%20.md).
本 PLAN 将该诊断设计转成可执行 gate；设计稿与本 PLAN 冲突时，以本 PLAN 的冻结合同、
任务顺序和验收条件为准。

## Goal

在不增加或重调任何 stabilization task、不改变 Phase 27 正常控制律的前提下，复现
current nominal MuJoCo 上 T0 static、T1 straight 与 T2 left/right 的首失效，并沿
`state error -> NMPC requested interaction wrench -> Minimal WBC realized wrench ->
joint/contact action -> MuJoCo acceleration/state` 逐层建立时序对齐的独立证据，最终对
T0 与 T1 各给出唯一且可复现的 first physical divergence mechanism；T2 只验证该机制
在左右转向下是否对称延续。

## Current State

- 已有：[Phase 27](../27-theory-restored-minimal-wbc/RECORD.md) 已完成 current-canonical
  wheel state/planner、16-state Eq.(12) NMPC、wheel-to-body internal interaction-wrench
  合同、42D/104-hard-row Minimal WBC、ProxQP 与 `2/10/20 ms` runtime 的 component、
  fault、replay 和 regression 验证。
- 已有：Phase 27 authoritative formal-v2 将 T0 首失效复现为 `0.58 s` safety envelope，
  T1、T2 left/right 首失效复现为 `0.45 s` safety envelope；这些结果是本 Phase 的
  不可覆盖 baseline，不因 diagnostic continuation 改写。
- 已有：current `StepResult` 与 `weighted_wbc_loop` control log 已记录 16-state NMPC
  status/audit、requested/realized 12D internal wrench、interaction residual/signed slack、
  42D WBC physical solution、task/resource/torque 与 contact/plant metrics。
- 缺少：原 nominal envelope 越界后的只读诊断续跑；NMPC predicted、WBC reduced-model
  与 MuJoCo actual acceleration 的同点、同 frame、同 sample 对齐；冻结状态 NMPC
  directional oracle；约束 active-set/resource 归因；growth-rate/frequency/phase-lag
  分析；能排除上游层后再判定 fast-mode gap 的决策树。
- Grounding：live-code authority 为 CBM project `W_L_ws`，generation
  `2026-08-29T06:47:42Z`；Core、16-state solver、WBC 与 runner 候选路径无 recorded
  coverage gap。`docs/`、`tools/` 按策略不入 CBM，已直接读取。Graphify 仅支持
  Phase 21～23 的历史 control/WBC/NMPC 关系，当前源码和 Phase 27 evidence 优先。

## Scope

- 以 Phase 27 authoritative formal-v2 的 T0、T1、T2 left/right 输入、seed、模型、
  controller/solver artifact、schedule、threshold 和 hash 重新建立 Phase 28 baseline，
  保存 first-failure tick/time/state 与完整 failure context。
- 建立 simulation-only、显式 opt-in 的 diagnostic continuation：原 safety envelope
  继续判定并记录 nominal failure，但仅该 envelope 的 latch/zero 动作可在诊断分支中
  延后至预冻结的 diagnostic termination envelope；正常 mode 完全不变。
- 增加 additive diagnostics、offline/shadow 计算、finite-difference/perturbation oracle、
  prediction-vs-plant residual、constraint/resource attribution 与 mode/growth analysis；
  所有诊断不得反馈到 plant command。
- 对 T0 分析 `theta/omega_y -> {F_Lx,F_Rx,T_Ly,T_Ry} -> predicted pitch
  acceleration`；对 T1 分析 longitudinal position/velocity error、common `Fx/Ty` 与
  predicted longitudinal acceleration；在冻结状态上执行正负小扰动 directionality oracle。
- 对同一控制 sample 比较 requested/realized 12D wrench、WBC interaction residual/slack、
  hard/soft residual、torque/contact/acceleration/workspace margin 与 active constraints，
  区分 realization failure 和资源饱和。
- 在同一 canonical point/frame 下形成 `a_NMPC`、`a_WBC`、`a_plant`，分别审计
  `r_upper=a_WBC-a_NMPC` 与 `r_plant=a_plant-a_WBC`；必要时分解 base/wheel、internal/
  contact、closed-chain 与 fast-joint 残差。
- 仅当 NMPC corrective direction、WBC realization 与 model-to-plant acceleration gates
  全部健康时，才执行更新率、增长率、主频、阻尼/相位滞后分析并考虑 fast low-level
  stabilization gap。
- 输出 T0/T1 primary attribution、T2 left/right symmetry check、限制与下一 Phase 的
  问题清单；本 Phase 不选择、增加或调参任何 WBC task。

## Out of Scope

- 增加 pitch、base-X、rolling、height、leg、orientation、wheel common/differential 或
  其他 WBC/NMPC/low-level stabilization task；task ablation、add-back、retuning、HQP、
  observer、自适应或 learning controller。
- 改变 NMPC/WBC 权重、OCP topology、state/input/reference、interaction-wrench semantics、
  42D decision、104 hard rows、ProxQP、contact cone、torque/acceleration/workspace limits、
  `2/10/20 ms` schedule、deadline、age、fault、fail-zero 或 reset 合同。
- 把 widened diagnostic envelope 当作新 formal acceptance threshold，或覆盖/重写
  Phase 27 的 Minimal FAIL、config、log、summary、manifest、REVIEW 或 RECORD。
- Phase 27 T3 wheel-differential OCP lifecycle/stationarity 修复；SQP-RTI robustness 另立
  后续 Phase。
- 真机、Hardware Adapter、STM32/树莓派、identified/CAD revision、terrain、传感器/
  执行器/接触辨识，以及任何 real-hardware claim。

## Frozen Decisions

- **Authority/non-overwrite：** Phase 27 formal-v2 与 replay/fault evidence 是 baseline
  authority。Phase 28 使用独立 namespace、config/schema 和空输出根，并通过
  `source_run/replay_of/supersedes` 指向旧证据；失败和无效 run 也追加保留。
- **Control law invariant：** current nominal plant、wheel planner、16-state model/Eq.(12)、
  acados solver、interaction-wrench affine map、Minimal soft set、42D/104-row WBC、ProxQP、
  torque extraction与 `2/10/20 ms` 全部逐项不变。新增值只可写入 diagnostics/log，
  不能进入 solver problem、reference、bounds、warm start 或 torque command。
- **Two-envelope semantics：** nominal envelope 仍在原 tick 生成不可撤销的
  `nominal_failure=true` 与原 failure enum。diagnostic continuation 仅允许在专用
  simulation runner 中推迟由 base pose envelope 导致的 zero/latch；model invalid、
  contact loss、non-finite、NMPC/WBC audit/solver、wrench age、hard/torque/workspace、
  deadline 与显式 fault injection 仍按 Phase 27 原 tick exact-zero/latch/stop。
- **Pre-freeze rule：** diagnostic termination envelope、maximum continuation duration、
  perturbation sizes、filter/window、numeric tolerance、recovery/bounded/drift/divergence
  判据与 attribution threshold 必须在任何 primary continuation/holdout 前写入 versioned
  method/config；不得根据 T0/T1/T2 结果调门槛。envelope 必须落在已批准 orientation
  chart、state/model-validity、contact与闭链 workspace 交集内，否则 DG28-01 REWORK。
- **Acceleration semantics：** 三层比较使用同一 pre-step state、同一 held command、
  canonical base-control point与 north/world 表达，并显式记录 sample/hold index。
  `a_NMPC` 来自冻结 16-state continuous model在 measured state/requested wrench 的
  derivative；`a_WBC` 来自 42D solution 的 reduced `nudot` 经已验证 kinematic map 到
  同一点；`a_plant` 以 MuJoCo object/site spatial acceleration为 primary，按 tick 对齐的
  twist finite difference为独立 oracle。不能直接比较不同点、frame或 pre/post-step 值。
- **Corrective-action oracle：** 每个 `+/-` state perturbation 使用相同 frozen state/
  reference、相同 solver reset/warm-start policy和相同非目标量；方向性同时由 wrench
  response与 model-predicted acceleration判断，不能仅凭单个 `Fx` 或 `Ty` 的符号下结论。
- **Attribution order：** nominal threshold-only → NMPC corrective action → WBC realization/
  resource → reduced-model-to-plant mismatch → fast-mode/bandwidth。后一层只有在前层以
  预冻结 gate 排除后才可成为 primary；若同时越界且无法建立时间先后/因果隔离，结论
  必须是 `unresolved/REWORK`，不得选择“最可能”类别完成 Phase。
- **Terminal outcome：** Phase PASS 不要求 Minimal 闭环稳定，也不批准某个新 task；
  它要求 T0、T1 分别获得唯一、证据闭合的 A～E primary mechanism，T2 仅给出一致/
  不一致与对称性结论。任何 task necessity 只可作为下一 Phase 的待验证假设。

## Open Questions / Decision Gates

- **DG28-00 / CLOSED PASS / CODEX — authority and reproduction contract：** 从 Phase 27
  formal-v2 manifest 固定 T0/T1/T2 left/right 的 exact input hashes、seed、duration、
  reference tick alignment、first-failure enum 与 replay tolerance；若不能 exact replay，
  先关闭环境/artifact差异，不进入诊断。
- **DG28-01 / CLOSED PASS / CODEX+EVIDENCE — diagnostic envelope：** 依据 Phase 27 nominal
  bounds、16-state `0.35 rad` attitude-chart有效域、MuJoCo contact/closure workspace 与
  offline envelope sweep，冻结逐轴 termination bounds和最大续跑时间。该 gate 必须在
  T04 实现与任何 continuation 输出之前关闭；不得把边界设到模型无效区。
- **DG28-02 / CLOSED PASS / CODEX+EVIDENCE — acceleration point/frame/timing：** 用静态、
  `+/-Fx`、`+/-Ty`、左右对称与 one-step corpus 关闭 NMPC derivative、WBC `nudot`、
  MuJoCo primary acceleration和finite-difference oracle的点、frame、sign、transport与
  pre/post-step alignment；不闭合则不得形成 `r_upper/r_plant`。
- **DG28-03 / CLOSED PASS / CODEX+EVIDENCE — attribution thresholds：** 在 primary data
  不可见条件下，以 Phase 27 component oracle error、floating-point floor和synthetic
  injected-failure corpus冻结 corrective score、wrench realization、acceleration residual、
  active-margin、growth/frequency与trajectory classification阈值。
- **DG28-04 / CLOSED PASS / EVIDENCE — first mechanism：** 按预冻结决策树分别关闭 T0/T1
  的 A `threshold-only`、B `NMPC corrective failure`、C `WBC realization/resource`、
  D `model-to-plant mismatch` 或 E `fast stabilization/bandwidth gap`；T2 只能支持或
  否定 T1 机制的左右对称延续。证据不能唯一分类时 REVIEW=REWORK。

## Interfaces and Compatibility

- 输入：canonical `RobotState`；Phase 27 versioned T0/T1/T2 reference/case；原
  16-state measured/reference state与左右 wheel-center internal interaction wrench。
- 内部：保持 `WheelState/Planner -> WheelAwareNmpcSolver -> requested interaction wrench
  -> Minimal WBC -> TorqueCommand`；旁路新增只读
  `DiagnosticSnapshot -> offline/shadow attribution`，不得回写主链。
- 输出：canonical `TorqueCommand` 不变；additive Phase 28 log/report 至少包含 nominal/
  diagnostic stop reason、三层 acceleration、requested/realized wrench、residual/slack、
  torque/contact/hard/soft margin、active-set、growth/frequency 与 attribution enum。
- 必须保持：Phase 21～25 default modes、Phase 27 normal mode、public I/O、plant、solver/
  artifact、timing、fault/fail-zero/reset、ordinary build与历史 evidence不变。
- 允许改变：仅 opt-in simulation diagnostic runner/config、additive `StepResult`/CSV/JSON
  fields、offline oracle/evaluator和 Phase 28 evidence；若无需扩展 public struct，优先
  在 runner 由现有状态/solution计算。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| T01 | 关闭 grounding 与 Phase 27 authority | design source、Phase27 RECORD/formal-v2、live Core/NMPC/WBC/runner | impact map、reuse/non-reuse表、exact baseline hashes、DG28-00 evidence | CBM+coverage+source/Graphify核对；fresh replay与原summary逐字段一致 | done |
| T02 | 冻结 diagnostic continuation 合同 | T01、nominal/model/chart/contact/workspace bounds | two-envelope spec、逐轴 termination bounds、duration、stop-priority、config/schema | DG28-01；边界预冻结且在模型有效域；normal mode zero/latch逐tick parity | done |
| T03 | 冻结 acceleration 与 corrective-action oracle | T01、16-state model、42D solution、MuJoCo state/acceleration API | point/frame/sign/timestamp spec、`a_NMPC/a_WBC/a_plant` maps、FD/perturbation corpus | DG28-02；equilibrium、±Fx/±Ty、左右/transport/one-step parity PASS | done |
| T04 | 实现 additive diagnostic runner/log | T02/T03、current weighted_wbc_loop/StepResult | opt-in continuation、nominal breach shadow record、diagnostic stop、additive snapshots/CSV/manifest | normal Phase27 mode bitwise/field parity；非-envelope faults仍exact-zero/latch；non-overwrite | done |
| T05 | 实现 evaluator并冻结判据 | T02～T04、Phase27 oracle errors | recovery/bounded/drift/divergence、corrective/realization/residual/resource/growth决策树与synthetic cases | DG28-03；`py_compile`；每类injected failure唯一命中，ambiguous输入输出REWORK | done |
| T06 | 执行 Gate 0 baseline reproduction | T01/T04/T05、fresh output root | T0/T1/T2-L/T2-R baseline、first-failure packages、replay diff | Phase27原threshold/first-failure/status/request/realization/safety context复现 | done |
| T07 | 执行 Gate 1 diagnostic continuation | T02、T06 | 四工况续跑time series、stop/max excursion、trajectory classification | nominal failure tick不变；每例只在diagnostic envelope或原hard/fault gate终止 | done |
| T08 | 执行 Gate 2 NMPC corrective attribution | T03/T05、T06/T07 first-divergence windows | T0 pitch与T1 longitudinal时序、冻结状态±perturbation oracle、corrective score | request和predicted acceleration方向一致性可判定；solver lifecycle与非目标量相同 | done |
| T09 | 执行 Gate 3 WBC realization/resource attribution | T05/T08、42D/logged contact/torque | request-realized 12D报告、residual/slack、hard/soft/torque/contact/workspace margin与active set | correct request若未实现，能以首个active resource或residual分类；否则明确排除C | done |
| T10 | 执行 Gate 4～6 plant/mode/symmetry attribution | T07～T09、三层acceleration | `r_upper/r_plant`报告、base/wheel/contact/closure分解；必要时growth/frequency/phase；T2 symmetry | 仅在前层PASS后进入下一层；T0/T1按DG28-04唯一分类，T2不扩张机制 | done |
| T11 | 执行 replay/fault/regression 与审查包 | T06～T10 authority | fresh replay、fault/reset/non-overwrite、Phase21～25与Phase27 regressions、classification matrix、REVIEW输入 | replay差异仅预声明wall-clock；normal fault/default parity；blocking finding=0 | done |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Preflight / build

- 在任何 formal 输出目录创建前执行
  `./.venv/bin/python -c "import mujoco, numpy, scipy; print(mujoco.__version__, numpy.__version__, scipy.__version__)"`；实际脚本使用 CasADi/acados 时同一命令一并探针。失败属于环境 gate，不得记为控制或模型 evidence FAIL。
- `./.venv/bin/python -m py_compile <phase28 oracle/evaluator/runner scripts>`：全部
  Python 入口先通过，才允许写稳定输出目录。
- 从 `ros_ws` 执行 Release build：
  `source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-up-to wheel_leg_mujoco --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DACADOS_ROOT=/home/t/opt/acados`。
- `cd ros_ws && source /opt/ros/jazzy/setup.bash && colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco && colcon test-result --verbose`：新增 component 与既有 suite 全部通过。

### Component / oracle

- Default parity：Phase 28 diagnostics关闭时，Phase 27 normal mode 的 command、status、
  fault tick、zero/latch/reset、NMPC age/update、WBC solution与既有日志字段一致。
- Two-envelope：nominal breach保持原 tick/axis/value；diagnostic runner只 shadow 允许的
  pose-envelope latch，所有其他 hard/fault reason的优先级与exact-zero不变。
- Acceleration：equilibrium、每侧 `+/-Fx`、`+/-Ty`、左右对称、point transport、
  one-step与finite-difference corpus关闭三层点/frame/sign/timing，禁止用轨迹拟合补偿差。
- Corrective oracle：T0 `theta/omega_y`、T1 position/velocity error各自的central
  perturbation、solver reset/replay与response score可重复；perturbation不越过state bound。
- Evaluator：synthetic A～E、ambiguous、missing/non-finite、nominal-recovery与hard-stop
  fixtures覆盖决策树；阈值/version/hash早于primary run。

### Formal / evidence

- Gate 0 对 T0/T1/T2-L/T2-R 逐 tick 对比 Phase 27 authoritative formal-v2；若首失效
  不复现，停止并调查 build/dependency/artifact/config/hash，不把差异当新物理结论。
- Gate 1 每例同时保存 nominal failure、diagnostic stop、post-failure最大 excursion、
  recovery/re-entry、bounded/drift/divergence 与完整原 hard/fault status。
- Gates 2～4 保存 divergence 前稳定窗口、首偏离窗口与首失效后窗口，统一记录
  state/reference、requested/realized wrench、slack、three-layer acceleration、torque/contact/
  residual/margin/active-set及sample alignment。
- Gate 5 只有在 B/C/D 排除后运行；分析区间、detrend/window、采样率、alias limit、
  growth fit 与置信判据按 DG28-03 冻结，不能将求解器/接触失效误判为fast mode。
- Gate 6 只比较 T2 left/right 对 T1 primary mechanism 的符号、发生层与时间尺度一致性；
  yaw/roll/asymmetric contact只能作为secondary finding，不能替代T0/T1 attribution。
- 每个 run 保存 interpreter/dependency、model/controller/solver/artifact/config/schema/seed/
  input hash、raw logs、summary、failure context与命令；fresh replay写新目录。

## Acceptance Criteria

- [x] T01～T11完成，DG28-00～04关闭，所有偏差与无效 run 追加保留。
- [x] Phase 27 T0/T1/T2-L/T2-R 的首失效在原 formal threshold 下可复现，历史判定和
  evidence未被覆盖。
- [x] diagnostic continuation的逐轴边界、时长、stop priority与分类阈值早于primary
  run冻结，且未越出16-state chart/model/contact/closure有效域。
- [x] diagnostics关闭时 Phase 27 normal control/fault/reset/replay保持一致；续跑只在
  simulation opt-in分支发生且不被宣称为production safety PASS。
- [x] `a_NMPC/a_WBC/a_plant` 已在同一state/sample/point/frame下由独立oracle关闭，
  requested→realized→plant链路无隐藏时序或符号换算。
- [x] T0、T1各自按预冻结顺序唯一归入A～E；若证据不足则REVIEW=REWORK，不以推测完成。
- [x] T2 left/right只给出T1机制的一致性/对称性结论；T3与任何task方案保持在范围外。
- [x] fault/fail-zero/reset、hard/solver/contact/deadline、fresh replay、non-overwrite与
  Phase21～25及Phase27 normal-mode regression全部通过并记录真实输出。
- [x] REVIEW无blocking finding；只有REVIEW=PASS后才创建RECORD并把ROADMAP标记complete。

## Execution Notes

按任务 ID 记录真实命令、结果、failed/inconclusive/superseded run 和 evidence 链接；
不建立第二份任务状态表。任何改变 control law、OCP/WBC 数学、hard constraint、solver、
timing、normal safety/fault语义或Phase27 threshold的发现，必须将 Phase 置为 REWORK/
blocked并修订PLAN，不能靠诊断开关、滤波、重新选窗口或调门槛继续。

## Blockers

None.

## Results

- `phase28-drift-attribution-v1` 保留 evaluator alignment FAIL；其 T0 direct-vs-FD
  angular RMS 为 `0.5018577075 rad/s²`，根因为 centered FD 覆盖两个 control interval，
  evaluator 当时只平均一个 interval 的五个 physics samples。门槛未改变。
- 修正为对称十个 substep 后，`v2` 关闭 alignment；随后只追加 evaluator 的
  ambiguous=`unresolved`、resource margin 与 T2 非 primary 语义，最终 authority 为
  `phase28-drift-attribution-v5`，fresh replay 为 `v6`。两者 480 个 control rows 除
  wall-clock 列零差异，plant 文件 byte-identical，summary exact-identical。
- T0/T1 nominal failure 分别精确复现 tick `58/45`；diagnostic stop 分别为 `70/62`。
  两例 corrective gate FAIL，而 request-realization、upper acceleration 与 plant
  acceleration gates PASS，因此唯一 primary mechanism 均为
  `B_nmpc_corrective_failure`。
- frozen-state oracle 复现 T0 pitch/pitch-rate 正反馈导数 `+118.153/+18.2632`；T1
  position/velocity 局部导数为 `-0.972159/-0.491522`，但 snapshot 净纵向加速度
  `-0.0118472 m/s²` 与负 position/velocity error 同向，确认 first-divergence window
  的净动作非恢复性，而非 WBC realization 或 plant mismatch。
- T2 不给 primary mechanism：right 与 T1 的 B 路径一致，left 在 B gate 不一致；结论
  仅为 `not_consistent`，未执行或声称 E 类 bandwidth 归因，也未触及 T3。
- Phase 27 normal regression 新输出逐字段（除 timing）等于 formal-v2，仍为预期的
  `Minimal FAIL`；全量 ROS component suite 为 `33 tests, 0 failures`，覆盖 fault、
  exact-zero/latch/reset 与 Phase 21～27 default contracts。REVIEW 见 [REVIEW.md](REVIEW.md)。
