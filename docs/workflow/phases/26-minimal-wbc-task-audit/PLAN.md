# Phase 26: current-nominal Minimal WBC task audit — PLAN

Status: `blocked`

Phase charter freeze: `2026-08-29` — user-approved scope, task crosswalk,
hard/soft boundary, audit-only safety policy, task sequence and acceptance
criteria. DG26-00～DG26-04 remain evidence gates; changing any frozen item
requires a recorded PLAN revision.

## Goal

在不连接真机、不改变 current nominal plant、canonical robot I/O、WBC 硬约束和故障 fail-zero 语义的前提下，建立可复现的平地 WBC task necessity audit：以真实 C++ NMPC→WBC 栈为对象，在受控测试包络内完成 baseline、冻结参数消融、Minimal 候选、重新调参与必要时 add-back，最终给出每项现有软任务的 A/B/C/D 归类以及 flat-ground `Minimal Stable WBC`；若 T1/T2 所需的直线速度或持续转向能力超出 Phase 23 已批准的 NMPC 合同，则以明确 decision gate 停止归因，不把安全锁存或上层能力缺失误判为 WBC task 失败。

## Current State

- 已有：[Minimal WBC 测试框架](../../../mujoco/minimal_wbc.md) 已冻结 `Current → frozen screening → Minimal → retuning → add-back` 的实验顺序、T0～T3 工况、hard/performance/wrench/task/resource 指标和 A/B/C/D 结论口径。
- 已有：[Phase 21](../21-nominal-weighted-wbc/RECORD.md) 冻结 current nominal 12-DoF、42-variable/104-hard-row WBC、contact-centred wrench、ProxQP 前身的 task/slack 语义、10 ms Core、2 ms physics、5-step ZOH、formal/fault/replay 入口；[Phase 22](../22-proxqp-solver-migration/RECORD.md) 只替换 WBC solver backend；[Phase 23](../23-nominal-nmpc/RECORD.md) 冻结 12D wrench producer、20 ms NMPC、2:1 更新和 straight small-reference authority。
- 当前 live C++ 的诊断枚举固定为 `Contact / BaseX / Height / Orientation / Leg / WrenchFidelity / SlackPenalty`，没有运行时 task mask 或独立权重 profile；其中 `WrenchFidelity + SlackPenalty` 是同一个 wrench-realization-with-slack block 的两项二次项，不是两个可独立删留的 Minimal task。文档中的功能 crosswalk 冻结为 `H→Height`、`P→Orientation.pitch`、`FX→BaseX`：`BaseX`以 X 位移/速度反馈生成基座 X 加速度目标，并经动力学与接触 wrench 分配承担 common-`F_x` 的水平速度追踪职责；它不是直接接触力残差。`R/WC/WD` 在 production WBC 中没有独立 task。
- 当前 `ControllerCore::stepWeightedWbc()` 在双轮接触、10 ms timing、x/y/z、roll/pitch/yaw、solver/hard residual、finite 和 torque limit 任一 gate 失败时六路 zero 并锁存。默认位姿包络为 `|x|≤0.02 m`、`|y|≤0.02 m`、`|z-z_nom|≤0.01 m`、`|roll|/|pitch|≤0.03 rad`、`|yaw|≤0.05 rad`；它会在 T1 长距离运动或 T2 转向中先于性能评价触发。
- 当前 `weighted_wbc_loop` 已有 nominal NMPC/Weighted-WBC 模式、`--torque-limit`、初态/腿扰动、逐 tick control/plant CSV、fault、replay 和 non-overwrite；现有日志含 42D physical solution、7 项 task residual/cost、slack、solver/hard、torque 和 contact/plant 指标。
- 缺少：文档 taxonomy 与 live task 的权威 crosswalk、可独立启停/调权的 versioned task profile、T0～T3 reference/case 合同、测试专用位姿包络与 production shadow gate、wheel common/differential 与 wrench/resource 指标、统一筛选/retuning/add-back evaluator 和 Phase 26 evidence。
- Grounding：CBM project `W_L_ws`、generation `2026-08-28T13:13:14Z`、full/ready；`docs/` 被主索引排除并已直接读取，`controller_core.cpp` 与 `weighted_wbc_loop.cpp` 的 coverage metadata changed，以上 live 事实以直接源码复核为准。

## Scope

- 只研究 current nominal full-3D、flat-ground、bilateral-contact MuJoCo profile 下现有 C++ NMPC→WBC 软任务的必要性；工况目标采用文档 T0 静止、T1 `0.20 m/s` 启动/匀速/制动、T2 `0.20 m/s` 与 `±0.08 rad/s` 持续转向、T3 `xi_delta(0)=±10 mm` 左右不对称恢复。
- 在任何消融前，冻结 production task ID、数学 residual、reference producer、document crosswalk 和 `N/A` 规则。不得为完成消融矩阵而先新增 `R/WC/WD`；production 中不存在的 task 只记录 `N/A`，除非 Minimal add-back 证据先证明缺失的是该控制职责，并经 PLAN 修订后实现。
- 为现有软任务增加内部、versioned、非 ROS 的 enable/weight profile；保持默认 profile 与 Phase 21/22/23 数学逐项一致。候选最小集合固定从 `Contact + WrenchRealizationWithSlack (current WrenchFidelity + SlackPenalty pair) + existing weak regularization` 起步；该 wrench block 不做内部消融。`BaseX/Height/Orientation/Leg` 是首轮真实消融对象；是否需要将 `Orientation` 或 `Leg` 细分到独立分量由 DG26-00 在实现前冻结。
- 复用 current solver、42D variable order、104 hard rows、model/contact/wrench map、warm/reset、Core/Adapter/runner 和正式 evaluator 结构；新增 task mask/profile、所需 reference profiles、日志字段、汇总指标和最小测试入口。
- 在 runner 显式选择的 audit-only profile 中放宽 **位姿工作域锁存阈值**，并同时计算 production strict-envelope shadow result；audit 阈值在 baseline 前一次性冻结，所有架构完全相同，写入 config/manifest/log/summary。
- 保持并验证所有硬约束与故障 gate：12-row dynamics equality、六路 torque box、两侧 37-row H-cone、12D acceleration box、workspace/reconstruction、bilateral contact、finite、timestamp/timing、solver/status/deadline、NMPC age/stale、hard residual、torque over-limit、fail-zero/latch/reset。
- 建立 T0→T1→T2→T3 early-stop、baseline-relative 与 near-zero absolute tolerance 评价；保存 config snapshot、逐 tick time series、per-case summary、failure reason/time/state、comparison table、manifest 和 hashes。
- 对最终候选执行 frozen、retuned、必要时定向 add-back/interaction，以及 fault/reset/fresh replay/non-overwrite/Phase 21–23 compatibility regression；结论只限 manifest 指定的 current nominal simulation host。

## Out of Scope

- 真机、STM32/树莓派、Hardware Adapter、identified/new CAD profile、传感器/执行器/contact 辨识、target-hardware realtime 或任何 real safety claim。
- 坡地、台阶、连续不平地形、地形法向变化、单轮支撑、腾空、大冲击、跌倒恢复和 terrain contact transition。
- 修改 canonical FLU、quaternion/world twist、joint order/sign/unit、公共 `RobotState/TorqueCommand`、ROS messages、Adapter contract 或 MuJoCo plant/contact/timestep。
- 改变 12-DoF WBC model、42D variable/slack order、104 hard rows、contact/wrench sign、ProxQP backend、NMPC/WBC schedule，或用 torque/contact/friction/normal-force/solver gate 放宽换取“稳定”。
- 预先执行 `2^N` 全组合、无证据的两两 task interaction、在线自动调权、gain scheduling、新 supervisor、fallback controller 或 last-valid torque/wrench hold。
- 把 Phase 23 的 small straight-reference PASS 自动外推到 `0.20 m/s`、continuous turning 或 360° yaw；把 viewer mouse perturbation 当作正式外扰证据。
- 覆盖或改写 Phase 21/22/23 config、generated artifact、formal run 或 evidence；所有 Phase 26 profile/run 采用新 namespace 和空输出目录。

## Frozen Decisions

- **Claim authority：** 本 Phase 回答“current C++ WBC 中实际存在的软任务在指定 flat-ground test envelope 内是否必要”。文档 taxonomy 的功能映射固定为 `FX→BaseX`、`H→Height`、`P→Orientation.pitch`；`R/WC/WD` 无现有实现时必须写 `N/A` 或使用真实 production task 名称，禁止借近似命名扩大结论。
- **No add-to-ablate：** 不为了证明某 task 可删除而先新增它。首个审计集合只含 live `BaseX/Height/Orientation/Leg`；`Contact/WrenchRealizationWithSlack/regularization` 构成 Minimal 候选的保留层。`WrenchRealizationWithSlack` 必须保持 current `WrenchFidelity + SlackPenalty` pair 的数学完整性，不单独关闭其中一项。新增控制职责只能由已保存的 Minimal failure attribution 与定向 add-back gate 触发。
- **Hard/soft boundary：** 42D/104-row hard problem、torque/H-cone/acceleration limits、workspace/reconstruction fail-closed 均不可 ablate。Phase 21 的 contact acceleration 仍按 soft task 处理，但它属于 Minimal 保留层；不得改写成未经验证的 rigid contact equality。
- **Frozen vs retuned：** Frozen screening 中 plant、NMPC/OCP/reference、solver、safety profile、其他 task 权重/gain、regularization 和 case seed 全部不变，只允许关闭目标 task 及删除失去意义的对应参数。Retuned 结果使用独立 profile/split，只允许调整剩余 WBC soft-task weight/gain/regularization候选；不得改 hard constraints、plant、NMPC、case 或 safety threshold。
- **Safety split：** production 默认配置和值保持字节/行为兼容。audit-only profile 只可放宽 `maximum_abs_x/y/z` 与 `maximum_abs_roll_pitch/yaw` 或采用等价的 reference-relative位姿 envelope；必须同时记录 strict production shadow-latch。所有架构共用同一 audit threshold，禁止逐候选放宽。
- **Safety always on：** invalid/non-monotonic/non-finite、bilateral contact loss、timing/deadline、solver/model/problem rejection、QP hard residual、workspace/reconstruction、torque hard limit、NMPC status/age/stale 和 fail-zero/latch/reset 永不关闭。`--torque-limit` 不是本 Phase 的调参旋钮，formal 使用冻结 production limits。
- **Failure classification：** `audit-envelope violation`、`strict-shadow-only violation`、`reference/model capability failure`、`NMPC failure`、`WBC solver/hard failure`、`physical/contact failure` 和 `closed-loop performance failure` 分开编码。strict shadow 越界但 audit envelope 内继续运行不等于 controller failure；audit envelope 越界仍必须 zero/latch。
- **Test ordering：** `Current baseline → frozen single/group screening → Minimal-Frozen → Minimal-Retuned → evidence-directed add-back → evidence-directed interaction`。每个架构按 T0→T1→T2→T3 early-stop；不存在证据时不运行组合。
- **Comparison：** hard absolute gate先于性能比较。误差类指标相对 Current 的 `±10% / 10–25% / >25%` 仅作预冻结筛选带；baseline 接近零的指标必须在 baseline 前冻结绝对 tolerance。不同 T0～T3 分表，不合成为单一总分。
- **Evidence/overwrite：** baseline、candidate、retuned、add-back、formal 和 replay 都使用 immutable config/hash 与新空目录；primary/replay除声明的 wall-clock字段外必须确定性一致。失败 run 保留并由新 run `supersedes`，不得原地改权重重跑。

## Open Questions / Decision Gates

- **DG26-00 / OPEN / CODEX — task contract：** 逐项冻结 live task matrix、normalization、weight、reference、diagnostic order，以及 document `R/WC/WD/H/P/FX` crosswalk。首选最小实现是现有 task 的 versioned mask/weight；只有 residual/attribution 必须独立时才拆分 `Orientation`/`Leg` 分量。该 gate 未关闭前不得运行消融。
- **DG26-01 / OPEN / CODEX+EVIDENCE — T0～T3 reference capability：** 证明现有 12D state/input/OCP 与小姿态 chart 能否在不改变 public I/O、physical model 和 WBC hard contract的情况下表达 T1 `0.20 m/s` profile、T2 `±0.08 rad/s` 及左右 360°。若 continuous yaw 需要改变 physical state/chart、OCP topology 或超出 validated contact/workspace，本 Phase 必须 REWORK 并建立前置 Phase；不得静默缩短 T2 或以 Phase 23 证据代替。
- **DG26-02 / OPEN / CODEX+EVIDENCE — audit safety envelope：** 在执行 Current baseline 前，用冻结 T0～T3 reference 路径、Phase 15 workspace 和 plant/contact limits确定一套统一 audit-only 位姿 envelope与 strict shadow 口径。必须证明 production default未变化、audit越界仍zero/latch、hard/fault gate未被旁路。
- **DG26-03 / OPEN / CODEX+EVIDENCE — metrics and thresholds：** 冻结 `xi_c/xi_delta` 定义、wrench desired/feasible/slack sign与左右聚合、primary `Fx/Fz/Ty`和完整12D报告、near-zero absolute tolerance、stable/early-stop/failure reason，以及 tuning/holdout split和有限retuning budget。看过holdout后不得修改 gate。
- **DG26-04 / OPEN / EVIDENCE — Minimal Stable WBC：** 只有 Current baseline、frozen screening、Minimal-Frozen、Minimal-Retuned及需要的定向 add-back全部按预声明矩阵完成，且 hard/fault/replay/compatibility gate通过后，才能对每项 task作A/B/C/D归类并选择最终集合。找不到更小稳定集合时，Current本身可以是证据支持的最小候选；不得强行删除 task。

## Interfaces and Compatibility

- 输入：canonical `RobotState`；现有 internal `NominalNmpcProblem/Reference`、`WbcReference`；versioned Phase 26 task/reference/safety/case profiles；10 ms WBC、20 ms NMPC、2 ms physics。
- 输出：canonical `TorqueCommand`保持不变；仅增加内部 task-profile选择与 additive diagnostics。正式产物为 per-tick control/plant log、per-case summary/comparison、manifest/hash 和 evidence note。
- 必须保持：public ROS/Adapter I/O、FLU/order/sign/unit、42D/104-row hard QP、WBC/NMPC solver identity与schedule、production默认task和safety行为、fault zero/latch/reset、旧profile可复现。
- 允许改变：Phase 26 opt-in test runner/config；现有 soft task 的内部 enable/weight；实现 T0～T3 所必需且经 DG26-01批准的内部 reference profile；additive metric/log/schema。任何新的 public command、physical state或OCP topology都需要 PLAN REWORK。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| P26-T01 | 冻结 live task taxonomy 与影响面 | minimal framework、Phase21–23 authority、live WBC/Core/runner | task matrix/crosswalk、hard-vs-soft边界、可干预项与`N/A`清单、grounding evidence | DG26-00；每个可关项有精确matrix/reference/residual，缺失项不伪映射 | todo |
| P26-T02 | 冻结 T0～T3 reference/state/case 合同 | P26-T01、Phase15 workspace、Phase23 OCP/reference | T0～T3 duration/profile/seed/initial condition、`xi_c/xi_delta`定义、reference capability oracle | DG26-01；T1/T2 model/chart/workspace可表达，否则REWORK并建立前置Phase | todo |
| P26-T03 | 实现最小 versioned task profile | P26-T01/T02、现有 `WeightedWbcProblem/Controller` | opt-in enable/weight profile、默认profile parity、disabled=`OFF` diagnostics、component tests | 默认 H/g/solution/torque/task结果与Phase22/23 baseline parity；single-disable只移除目标objective项 | todo |
| P26-T04 | 实现 audit-only 位姿包络与 shadow production gate | P26-T02、current Core safety | opt-in envelope profile、failure enum/diagnostics、strict-shadow记录、Core/runner tests | DG26-02；production default parity；audit-only位姿阈值可放宽；audit越界及所有hard/fault仍zero/latch/reset | todo |
| P26-T05 | 扩展 runner/log/evaluator 与正式方法 | P26-T02～T04、现有 formal runner | Phase26 config/schema、T0～T3 runner、wrench/task/resource/performance指标、early-stop、comparison与manifest | dependency probe+`py_compile`；synthetic metric oracle；non-overwrite；failure分类与OFF字段准确 | todo |
| P26-T06 | 执行 Current baseline 与冻结阈值 | P26-T05、未见candidate结果的frozen config | Current×T0～T3 tuning/holdout baseline、near-zero absolute tolerances、strict-shadow统计 | DG26-03；所有hard gate与reference capability PASS；baseline/replay可复现 | todo |
| P26-T07 | 执行 frozen-parameter screening | P26-T06、task mapping | 首轮 `-BaseX (FX)/-Height/-Orientation/-Leg`及证据支持的group/component runs、early-stop failure packages | 非目标参数/hash不变；每个结论同时含tracking、wrench/slack、task、resource、solver/hard/contact指标 | todo |
| P26-T08 | 构建 Minimal、retune 与定向 add-back | P26-T07、预冻结 tuning/holdout/budget | Minimal-Frozen、Minimal-Retuned、必要add-back/interaction run及候选集合 | 未见holdout不调gate；add-back只针对已归因缺失职责；Current vs final逐T0～T3比较 | todo |
| P26-T09 | 执行 formal/fault/replay/compatibility | P26-T08 final candidate、Phase21–23 regressions | 新formal+fresh replay、fault/reset/non-overwrite、Phase21/22/23 default parity与历史回归 | hard/solver/contact/plant/deadline/fault全部PASS；production default不受audit profile影响；hash审计PASS | todo |
| P26-T10 | 审查并记录 task 分类 | P26-T06～T09 evidence | A/B/C/D/N/A比较表、Minimal Stable WBC集合、限制/后续项、REVIEW输入 | DG26-04；结论逐项可追到immutable run；blocking finding为零方可进入REVIEW PASS | todo |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Preflight and build

- `./.venv/bin/python -c "import mujoco, numpy, scipy; print(mujoco.__version__, numpy.__version__, scipy.__version__)"`：在任何 formal 输出目录创建前记录依赖；Phase 26 新脚本需要 CasADi/acados 时一并探针。
- `./.venv/bin/python -m py_compile <phase26 scripts>`：所有 evaluator/oracle/runner wrapper 语法通过。
- `cd ros_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-up-to wheel_leg_mujoco --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DACADOS_ROOT=/home/t/opt/acados`：从 `ros_ws` 构建，普通 build 不运行 code generation。
- `cd ros_ws && source /opt/ros/jazzy/setup.bash && colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco && colcon test-result --verbose`：新增 task/safety/component 测试和既有 suite 全部通过。

### Component / oracle

- 默认 task profile 对 Phase 22/23 golden problem 的 `H/g/A/l/u`、solution、torque、diagnostics逐项 parity；禁用单 task 只移除其 objective contribution，42D/104-row hard problem不变。
- safety test覆盖 strict default、audit override、strict shadow、audit越界、contact loss、invalid、nonmonotonic、timing、solver/nonfinite/hard/torque/NMPC age fault 与 reset。
- metric oracle覆盖 `xi_c/xi_delta`、desired/feasible/slack、12D wrench channel、OFF residual、torque/friction/normal margin、relative与near-zero absolute比较、early-stop/failure classification。
- DG26-01 reference oracle必须先于正式 baseline；T1/T2 任何 model/chart/workspace gate失败都不是 WBC task failure。

### Formal / evidence

- 用冻结 Phase 26 config 在新目录执行 Current baseline、frozen screening、Minimal-Frozen、Minimal-Retuned和证据要求的add-back；每个run记录 interpreter/dependency、model/profile/controller/solver/reference/task/safety/case hash。
- T0～T3分别输出 hard validity、closed-loop performance、WBC realization、task residual、control resource和failure classification；不使用单一综合分数。
- 最终候选执行现有 fault类型、双episode reset、fresh replay、non-overwrite和Phase21/22/23 default-profile compatibility；primary/replay只允许预声明wall-clock字段不同。
- expensive/full matrix必须遵守 early-stop，但停止的case仍保存 failure time/reason/state、constraint、wrench、task和resource上下文。

## Acceptance Criteria

- [ ] P26-T01～T10完成，且无未记录偏差或越界扩张。
- [ ] document taxonomy与live task建立精确crosswalk；不存在的`R/WC/WD`未被伪实现或伪消融，`FX→BaseX`按其间接的水平速度/共同前后向 wrench 作用解释。
- [ ] T0～T3 reference capability在正式消融前关闭DG26-01；若不能关闭，Phase以REWORK停止而非缩减工况后宣称PASS。
- [ ] production默认task、安全阈值、42D/104-row硬约束、solver/schedule和public I/O保持兼容。
- [ ] audit-only位姿包络已显式放宽并固定用于所有架构；strict production shadow可见；torque/contact/friction/normal/solver/fault gate从未放宽。
- [ ] Current、Frozen、Minimal-Frozen、Minimal-Retuned及必要add-back有immutable config、summary、time series、manifest/hash和可追踪比较。
- [ ] 每个现有辅助task得到A/B/C/D归类，缺失task得到N/A；结论同时考虑tracking、wrench/slack、task competition、resource、hard/contact和solver证据。
- [ ] 最终集合在声明的flat-ground T0～T3范围满足hard gate和预冻结性能口径；若最终集合等于Current，证据明确说明未找到可删除项。
- [ ] fault/reset/fresh replay/non-overwrite、Phase21/22/23 default parity和历史回归通过。
- [ ] REVIEW无blocking finding后才创建RECORD并将Phase/ROADMAP改为complete。

## Execution Notes

按任务 ID 记录真实命令、结果、失败 run、偏差和 evidence 链接。Tuning 与 holdout 输入必须先写入 versioned config；任何 threshold/profile 修改都新建 run 并写 `supersedes`。本 Phase 不维护第二份任务台账。

- 2026-08-29：用户明确要求优先执行 [Phase 27](../27-theory-restored-minimal-wbc/PLAN.md)。本 Phase 当前只有 PLAN，P26-T01～T10 均为 `todo`，没有源码、配置、runner 或 evidence 可交接。由于 Phase 27 将改变 physical NMPC state、interaction-wrench semantics 与候选 schedule，触发 DG26-01 的“前置 Phase”条件；Phase 26 保留为 current-12D task audit，但在 Phase 27 给出 terminal contract/outcome 前不继续，也不把 Phase 27 结果计作本 Phase task evidence。

## Blockers

- `2026-08-29 / BLOCKED`：按用户优先级先执行 Phase 27；Phase 26 的 current-12D T1/T2 capability、task necessity 与阈值不能跨 physical state/OCP/interface revision 继续推断。恢复时从 P26-T01/DG26-00 重新 grounding，并先根据 Phase 27 RECORD 修订 DG26-01。
