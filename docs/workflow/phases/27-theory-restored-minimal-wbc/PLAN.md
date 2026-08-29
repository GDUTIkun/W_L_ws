# Phase 27: theory-restored wheel-aware NMPC + Minimal WBC — PLAN

Status: `complete`

Handoff: the retired current-12D audit draft contained only its PLAN at the
user-directed 2026-08-29 handoff; all proposed tasks were `todo`, so no
implementation/evidence was inherited. Its task-necessity conclusions and
thresholds did not transfer to this Phase because the upper model, wrench
semantics and candidate timing are different. The empty draft was later
deleted by user direction; this historical statement is retained without a
dead link.

Design source: [原理论模型恢复与 Minimal WBC 验证方案 v2](../../../mujoco/minimal_wbc_theory_restore_phase_plan_v2.md).
This PLAN is the workflow authority and converts that design draft into
executable gates; where the draft
still names a candidate rather than an approved contract, the item remains a
decision gate below.

## Goal

在 current nominal full-3D MuJoCo plant 上，以追加式 revision 恢复 common
wheel-position planner、16-state wheel-aware NMPC 与 paper Eq.(12) wheel-relative
dynamics，建立原 wheel-to-body interaction wrench 到当前 42D/6D-contact WBC 的
可审计物理合同，并验证只含 interaction-wrench realization、soft contact
acceleration 与弱正则项的 Minimal WBC 能否完成平地 T0～T3 闭环；若不能，输出
可复现的首失效层归因，不在本 Phase 增加补偿 task。

## Current State

- 已有：[Phase 23](../23-nominal-nmpc/RECORD.md) 已完成 12-state
  locked-composite、12D external base-control-point wrench、20 ms/N=20 acados
  SQP-RTI+HPIPM、10 ms WBC、2 ms physics 的 current-nominal 闭环。该版本作为
  非覆盖 baseline 保留，不是本 Phase 的 16-state generated artifact。
- 已有：[Phase 23 model-closure decision](../23-nominal-nmpc/evidence/model-closure-decision.md)
  已证明历史 16D 输入是 wheel-to-body internal interaction wrench，而当前
  `WbcReference` 是 contact-centred wrench 经 lever arm 运输到 base-control point
  的 external resultant；两者仅维数相同，直接代入会混淆 actor/receiver 并重复
  lever arm。本 Phase 必须以新合同关闭该拒绝理由，不能推翻记录或变量改名绕过。
- 已有：[Phase 23 state contract](../23-nominal-nmpc/evidence/state-contract.md)
  已验证 canonical base pose/twist 与 `xi_L/xi_R/dxi_L/dxi_R` diagnostic map；
  Simulink 基线已有 `full_base_nmpc_state_signal.m`、
  `full_base_body_dynamics.m`、`wheel_position_governor_step.m` 和
  `wheel_interface_wrench_contract.m`，但其 Euler chart、历史参数和故障 hold
  语义不是 production authority。
- 已有：[Phase 21](../21-nominal-weighted-wbc/RECORD.md) 与
  [Phase 22](../22-proxqp-solver-migration/RECORD.md) 已冻结 12-DoF reduced model、
  42-variable/104-hard-row QP、每轮 6D contact-centred wrench、ProxQP、torque/
  acceleration/contact bounds、workspace fail-closed 与 fail-zero/latch/reset。
- 当前 live `NominalNmpcModel::kStateSize=12`；`WbcReference` 同时含 base/height/
  orientation/leg acceleration targets 和 12D `interaction_wrench_flu`；
  `WeightedWbcProblem::assemble()` 当前用 `wrench_flu_map * w_C - slack` 直接拟合
  external reference；`ControllerCore::stepNominalNmpcWbc()` 每两个 10 ms WBC
  tick 更新一次 NMPC。
- 缺少：current-canonical wheel planner、获批准的 16-state contract、原 internal
  interaction-wrench 精确定义、对 42D decision 的 realized-interaction-wrench
  仿射重建、对应 oracle、新 acados artifact、Minimal-WBC profile、新调度 evidence、
  T0～T3 formal 与层级 failure attribution。
- Grounding：CBM project `W_L_ws`，generation `2026-08-28T13:13:14Z`，
  full/ready；`docs/` 按策略未索引并已直接读取。`nominal_nmpc_solver.hpp`、
  `controller_core.cpp`、`weighted_wbc_loop.cpp` 的 coverage freshness 为 changed，
  本 PLAN 的相关事实已用当前源码复核；其余列出的 Core/WBC/Simulink 候选路径
  无 recorded gap，但该信号仍是 best-effort。

## Scope

- 冻结 canonical `xi_L/xi_R`、common/differential coordinate、速度、planner
  reference/governor、左右顺序、frame、sign、reset 与工作域合同；`xi` 是 wheel
  center 相对 canonical base-control frame 的前向几何位置，不是 wheel spin angle。
- 冻结并实现新的 16-state wheel-aware NMPC revision：base position/orientation/
  twist 12 states + `xi_L/xi_R/dxi_L/dxi_R`，恢复 Eq.(12) wheel-relative dynamics，
  使用左右 12D 原 interaction-wrench input、20 ms sample、N=20 和新 namespaced
  acados artifact。
- 冻结 historical interaction wrench 的 actor/receiver、point、frame、sign、
  left/right 与 `[Fx,Fy,Fz,Tx,Ty,Tz]` order；从 WBC decision 的 `nudot` 与每轮
  6D contact-centred wrench 经刚体 Newton–Euler balance 重建实际 interaction
  wrench，形成固定 `q,nu` 下的仿射映射与独立 oracle。
- 保持当前 12-DoF/42D/104-row WBC foundation、ProxQP 和全部硬安全合同；建立
  opt-in Minimal profile，仅保留 interaction-wrench fidelity+signed slack penalty、
  soft contact acceleration 和弱 `nudot/tau/w_C` regularization。
- 冻结能被整数 ZOH 表达的 physics/WBC/NMPC 周期、deadline、age、reset 与 replay
  语义；首选候选为 1/5/20 ms，但只有通过 DG27-04 后才可进入 production candidate。
- 逐 Gate 完成 planner、16-state model、OCP、interaction interface、Minimal WBC
  algebra、runtime 和 T0～T3 闭环；所有 formal 使用 immutable config、manifest、
  hashes、新空输出目录与 append-only `supersedes/replay_of` 关系。
- 若 Minimal 闭环失败，修复本 Phase 范围内的 planner/model/OCP/interface 结构错误；
  对已证明结构正确后的 WBC realization、contact/plant 或 fast low-level 缺口只做归因，
  不增加补偿任务。

## Out of Scope

- 真机、Hardware Adapter、STM32/树莓派、identified/CAD revision、传感器/执行器/
  接触辨识，以及任何 real-hardware 或 terrain claim。
- 把历史 Simulink generated code、Euler state map、参数、bounds、last-valid hold 或
  36D single-point-force WBC 直接复制为 production authority。
- 把 NMPC internal interaction wrench 直接解释为 WBC contact-centred wrench，或只用
  rotation/point shift 代替 internal↔external Newton–Euler balance。
- 改变 public canonical `RobotState -> TorqueCommand`、ROS messages、Adapter contract、
  joint order/sign/unit、current nominal MuJoCo plant 或 ProxQP backend。
- add-back/ablation 补偿 task、height/pitch/roll/yaw reset/base-X/leg posture/wheel
  common/wheel differential task、自适应权重、HQP、slack feedback、observer、terrain、
  learning/RL 或新的 low-level controller。
- 用放宽 torque/contact/friction/normal-load/solver/workspace/fault gate、缩小预声明
  工况或覆盖 Phase 21～25 evidence 的方式获得 PASS。

## Frozen Decisions

- **Revision/non-overwrite：** Phase 23 12-state solver、config、formal 和 claim 全部
  保留；16-state model、generator、artifact、runtime mode、config、schema 与 evidence
  使用 Phase 27 namespace。普通 build 只编译 checked-in artifact，不运行 generation。
- **Architecture boundary：** canonical public I/O、current nominal plant、12-DoF reduced
  WBC model、42D order
  `z=[nudot_12,tau_6,wL_C6,wR_C6,sL_I6,sR_I6]`、104 hard rows、每轮 6D
  contact-centred wrench与ProxQP不变。
- **Interaction quantity：** NMPC target quantity 是 wheel follower 对 leg/base 的
  wheel-to-body internal wrench，目标作用点为对应 wheel-center，controller body/FLU
  表达，left block 在前，单侧 order 为 `[Fx,Fy,Fz,Tx,Ty,Tz]`。该定义必须由
  DG27-02 的 action-reaction、frame、point 与历史 source oracle 关闭后才成为代码
  authority；若证据否定它，PLAN 必须 REWORK，不得换名继续。
- **No direct substitution：** `W_NMPC^I != w_C`。WBC 必须实现
  `W_real^I=A_I(q,nu)z+b_I(q,nu)` 和 residual
  `r_W=W_real^I-W_NMPC^I-s_I`；零 residual 的 signed-slack 语义是
  `W_real^I=W_NMPC^I+s_I`。任何 nonlinear-in-decision 结果都触发 PLAN REWORK，
  不能静默改变 QP problem class。
- **Wheel dynamics：** Eq.(12) 的 `Fx/Ty` 只读取冻结的 internal interaction
  wrench；contact-centred `Fr/Ml` 不得直接代入，任何 point lever arm 只计算一次。
- **Minimal soft set：** 仅保留完整的 interaction-wrench fidelity/slack pair、
  current soft contact acceleration 和弱 regularization；base-X、height、orientation、
  leg、wheel common/differential 及其他 corrective state feedback 全部关闭。hard
  constraints、workspace/fault gate不属于消融项。
- **Solver/OCP family：** acados SQP-RTI + partial-condensing HPIPM、`Ts=20 ms`、
  `N=20`；state/reference/bounds/cost/dynamics 必须针对 16-state revision 从批准的
  symbolic/RK4 model 重新生成并做 project-owned defect/bound/stationarity audit。
- **Evidence order：** Gate 1 wheel contract → Gate 2 wrench/model contract → Gate 3
  OCP → Gate 4 Minimal WBC algebra → Gate 5 full closed loop。前一 Gate 未 PASS 时不得
  用后一层轨迹或调权关闭问题。
- **Valid terminal outcomes：** controller-level `Minimal PASS` 与证据充分的
  `Minimal FAIL + first-failure-layer attribution` 都可完成本 Phase；若 planner、
  model、OCP 或 interface 仍有未解决结构错误，则 REVIEW 必须 REWORK。

## Open Questions / Decision Gates

- **DG27-00 / CLOSED / CODEX — retired audit handoff：** 2026-08-29 用户明确选择先执行
  Phase 27；此前的 current-12D audit 草案仅有 PLAN、全部 proposed tasks 为 `todo`，
  没有 task profile、日志、evaluator、case、源码或 evidence 可继承。可复用面因此仅限
  Phase 21～23 已批准的 live Core/WBC/runner 基线；current-12D task necessity/threshold
  claims 不迁移。该空草案后来按用户要求删除并退役。
- **DG27-01 / CLOSED PASS / CODEX+EVIDENCE — wheel state/planner：** 以 current reconstructed
  geometry 关闭 `xi/dxi` 左右顺序、base-control origin、body/world表达、旋转 frame
  速度项、common/differential 定义、governor limits、continuity、finite difference、
  workspace 和 reset；历史 Simulink 数值只作 oracle 输入。
- **DG27-02 / CLOSED PASS / CODEX+EVIDENCE — interaction-wrench closure：** 从 Simulink
  actor/receiver contract、wheel/body Newton–Euler balance 与 current WBC 变量推导
  `A_I,b_I`；关闭 action-reaction sign、point/frame transport、component order、
  no-double-lever-arm、virtual-work/independent oracle 和仿射性。该 gate 是恢复
  Eq.(12) 与接 16-state NMPC 的共同前置。
- **DG27-03 / CLOSED PASS / CODEX+EVIDENCE — 16-state attitude/model：** 冻结 orientation
  chart/rate 定义、continuous yaw/turning有效域、current parameters、Eq.(12)
  `m_b,m_w,rho,I_w`、state/reference/bounds 和 analytic/RK4 sensitivities。不得在
  未证明 continuous turning 的 chart 语义时沿用历史 Euler rates 或外推 T2。
- **DG27-04 / CLOSED PASS / CODEX+EVIDENCE — timing：** 比较保持 2/10/20 ms 与候选
  1/5/20 ms 对 plant数值稳定、contact、WBC/NMPC deadline、ZOH、日志/扰动时序和
  replay determinism 的影响。只有同一 interpreter/build 的 component+closed-loop
  evidence 才可冻结新 schedule；不能因理论周期偏好直接改 plant timestep。
- **DG27-05 / CLOSED PASS / CODEX+EVIDENCE — OCP/reference/cases：** 在看 formal 结果前
  冻结 planner target、weights/bounds/rate limits、tuning/holdout、T0 static、T1
  `0.20 m/s` start-cruise-brake、T2 `0.20 m/s` + `±0.08 rad/s` bounded continuous
  turn 和 T3 `xi_delta(0)=±10 mm` 的 duration/seed/threshold。若 T2 需超出批准
  attitude/contact/workspace envelope，先 REWORK 工况/模型合同，不把它记为 WBC FAIL。
- **DG27-06 / CLOSED MINIMAL FAIL / EVIDENCE — final outcome：** 所有 upstream Gate 与 formal/
  fault/replay 完成后，判定 Minimal PASS；或在 hard gates健康的前提下，给出 planner、
  NMPC model、NMPC OCP、NMPC→WBC interface、WBC realization、contact/plant、
  suspected fast low-level stabilization 中唯一首失效层及支持证据。

## Interfaces and Compatibility

- 输入：canonical `RobotState`；Phase 27 versioned motion/planner reference；冻结的
  16-state internal problem与左右 wheel-center interaction-wrench contract。
- 内部：`WheelState/Planner -> Nmpc16Problem/Solver -> InteractionWrenchRequest ->
  MinimalWbcProblem/Solution` 或语义等价且可独立测试的边界；不能复用同名 12-state
  artifact 隐藏 dimension/semantics 变化。
- 输出：canonical `TorqueCommand` 不变；仅增加 opt-in runtime mode 与 additive
  diagnostics：planner state/reference、requested/realized internal wrench、fidelity、
  signed slack、contact-centred wrench、model/OCP/solver/timing/failure layer。
- 必须保持：Phase 21/22/23 default mode、public I/O、42D/104-row hard contract、
  ProxQP、contact/torque/acceleration/workspace gate、fail-zero/latch/reset 与 ordinary
  build compatibility。
- 允许改变：仅 Phase 27 namespace 内的 internal state/OCP/artifact、wrench-fidelity
  affine map、Minimal soft profile、经 DG27-04 批准的 opt-in schedule、runner/config/
  evaluator/log schema。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| T01 | 关闭 grounding 与 retired audit handoff | design v2、Phase21～25与retired draft、live Core/WBC/runner | impact map、reuse/non-reuse清单、DG27-00 evidence、version/ownership清单 | CBM+coverage+source/Graphify交叉核对；无覆盖旧artifact/evidence | done |
| T02 | 冻结 wheel-state 与 planner contract | current reconstruction、Phase23 xi oracle、Simulink planner | `xi_L/R,c/delta` spec、planner/governor、golden vectors、reset/workspace oracle | DG27-01；sign/order/frame/finite-difference/continuity/limits/replay全部PASS | done |
| T03 | 推导 interaction-wrench contract 与仿射重建 | historical contract、current reduced model、42D decision | actor/receiver/point/frame/sign spec、`A_I,b_I`推导、implementation-independent oracle corpus | DG27-02；±Fx/±Fz/±Ty、symmetry、action-reaction、transport round-trip、virtual-work、slack sign、affine parity PASS | done |
| T04 | 冻结 16-state continuous/RK4 model | T02/T03、current nominal parameters、Eq.(12) | state/input/chart/reference/bounds/model spec、analytic Jacobian/RK4 implementation与oracle | DG27-03；equilibrium、one-step、FD sensitivity、common/differential、left/right、model-validity PASS | done |
| T05 | 冻结 schedule 与 deadline | current 2/10/20 baseline、1/5/20 candidate、T04 cost | timing decision record、phase/ZOH/age/reset/deadline contract、numerical comparison | DG27-04；plant/contact stability、integer schedule、WBC/NMPC/combined timing与replay PASS | done |
| T06 | 生成并验证 16-state acados OCP | T04/T05、frozen OCP/reference profile | generator、checked-in namespaced artifact、C++ wrapper、model/OCP component evidence | clean双生成hash审计；generated next/A/B parity；equilibrium/±reference/brake/return/wheel/differential/bounds/defect/stationarity/timing/reset PASS | done |
| T07 | 实现 Minimal 42D WBC profile | T03、current model/QP/ProxQP与既有profile infra | affine realized-wrench task、signed slack、contact+regularization-only profile、diagnostics/golden problems | hard matrix/order unchanged；independent algebra/solver parity、equilibrium/dynamic corpus、torque extraction/deadline PASS | done |
| T08 | 集成 opt-in runtime 与日志 | T02/T05～T07、Core/Adapter/runner | planner→16D NMPC→Minimal WBC mode、schedule、fail-zero、additive control/plant logs、manifest | production default parity；phase/ZOH/age/fault/reset/non-overwrite component tests PASS | done |
| T09 | 冻结 formal 方法与门槛 | DG27-01～05 evidence、T0～T3 candidate | versioned method/config/schema、tuning/holdout、hard/performance/NMPC/WBC/resource metrics、failure enum | DG27-05；synthetic evaluator oracle；threshold冻结早于holdout；`py_compile` PASS | done |
| T10 | 执行 T0～T3 formal 与归因 | T08/T09、fresh output roots | primary runs、per-tick/summary/manifest/hash、first-failure packages、candidate conclusion | hard gate先行；planner/NMPC/interface/WBC/contact指标齐全；DG27-06可判定 | done |
| T11 | 执行 fault/replay/regression 与审查包 | T10 outcome、Phase21～25 regressions | fault/reset/fresh replay/non-overwrite、default-mode regressions、REVIEW输入与后续问题 | exact zero/latch/reset；replay差异仅预声明wall-clock；历史authority不变；blocking finding=0 | done |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Preflight / build

- 在任何 formal 输出目录创建前执行
  `./.venv/bin/python -c "import mujoco, numpy, scipy, casadi; print(mujoco.__version__, numpy.__version__, scipy.__version__, casadi.__version__)"`，并用同一解释器探针 Phase 27 generator 实际需要的 `acados_template`；失败属于环境 gate，不是模型 evidence FAIL。
- `./.venv/bin/python -m py_compile <phase27 generator/oracle/evaluator scripts>`：
  所有 Python 入口通过后才允许 formal 写入稳定目录。
- 从 `ros_ws` 执行 Release clean build：
  `source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-up-to wheel_leg_mujoco --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DACADOS_ROOT=/home/t/opt/acados`。
- `cd ros_ws && source /opt/ros/jazzy/setup.bash && colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco && colcon test-result --verbose`：新增 component 与既有 suite 全部通过。

### Component / oracle

- Wheel contract：canonical reconstruction 对独立 MuJoCo/geometry oracle、position/
  velocity finite difference、left/right/common/differential、governor saturation/reset。
- Wrench contract：equilibrium、每侧/每关键通道双向脉冲、action-reaction、point/frame
  round trip、no-double-lever-arm、Newton–Euler/virtual-work、`A_I z+b_I` parity 与
  `W_real-W_request-s=0`。
- NMPC：continuous/RK4/generated next 与 A/B parity、equilibrium、common/differential、
  symmetry、OCP bounds/defect/projected stationarity、cold/warm/reset/deadline。
- Minimal WBC：42D/104-row hard parity、只保留三类 soft block、QP/ProxQP independent
  oracle、representative states、wrench/slack/contact/torque diagnostic order。
- Runtime：冻结 phase/ZOH/age、all fault classes、six-zero latch/reset、default Phase23
  mode parity 和 output non-overwrite。

### Formal / evidence

- T0 static、T1 start-cruise-brake、T2 left/right bounded continuous turn、T3
  `xi_delta=±10 mm`；每个 case 保存 config/profile/model/controller/solver/artifact/
  schedule/seed/input hash、dependency versions、逐 tick log、summary 和 failure context。
- Hard gate至少覆盖 completion/stability、NMPC/WBC status/fault ratio、deadline、QP
  feasibility、dynamics residual、torque/contact cone/normal load/workspace、finite 与
  fail-zero；hard gate失败不能用 tracking 结果覆盖。
- 性能分层记录 base/twist/attitude、`xi_c/xi_delta`、NMPC defect/KKT/bounds/wrench rate、
  requested/realized interaction wrench/fidelity/slack、contact-centred wrench、soft
  contact residual、torque/contact resource margin。
- Minimal FAIL 时保存 first failure time/state、planner、NMPC、requested/realized
  wrench、slack、active constraints、contact truth 与唯一 first-failure layer；不得在
  同一 Phase 运行 add-back task。
- final outcome 执行 fresh-process replay、fault/reset、non-overwrite 与 Phase 21～25
  default-mode regression；任何失败 run 保留并以新 run `supersedes`。

## Acceptance Criteria

- [x] T01～T11完成，DG27-00～06关闭，所有范围偏差有记录。
- [x] Phase 23 拒绝历史 16D candidate 的 internal/external、point/frame、lever-arm
  问题已由新物理合同与独立 oracle 实质关闭，而非由变量改名或维数相同推断。
- [x] Wheel planner、16-state state/input/chart/Eq.(12)、OCP/reference/bounds、timing 与
  T0～T3合同均在 implementation/formal 前冻结并有 versioned evidence。
- [x] 新 acados artifact 可从批准模型确定性生成，ordinary build 不生成代码；
  generated model/OCP audit、deadline 与 reset通过。
- [x] Minimal WBC 保持42D/104-row/ProxQP及全部hard/fault合同，只含批准的 wrench
  realization+slack、soft contact acceleration与弱regularization；无隐藏 state task。
- [x] current Phase21～23 default mode、public I/O、plant、solver与历史 evidence 未被
  覆盖，既有 profile infrastructure 的复用边界可审计。
- [x] T0～T3 primary、fault、fresh replay、non-overwrite、历史回归具有 immutable
  config、time series、summary、manifest/hash 与真实命令/结果。
- [x] 若 controller-level Minimal PASS，全部预冻结 hard/performance/timing/replay gate
  同时通过；若 controller-level Minimal FAIL，upstream结构gate已通过并给出证据充分
  的唯一首失效层与下一 Phase 问题。
- [x] REVIEW无 blocking finding；只有 REVIEW=PASS 后才创建 RECORD 并把 ROADMAP
  标记 complete。

## Execution Notes

按任务 ID 在本文件记录真实命令、结果、failed/superseded run 与 evidence 链接；不建
第二份任务台账。任何改变 actor/receiver、state chart、Eq.(12)输入、QP problem class、
public I/O、plant、hard constraints、solver family 或已冻结 T0～T3 合同的发现都先将
Phase 置为 REWORK/blocked 并修订 PLAN，不能靠调权或补偿 task 继续。

- 2026-08-29：用户明确要求执行 Phase 27；此前 current-12D audit 草案的前置条件因此
  不成立，本 Phase 转 `active` 并关闭 DG27-00。该草案没有实现或 evidence 可交接，
  本 Phase 从 Phase 21～23 approved baseline 重新 grounding；空草案后来按用户要求
  删除并退役。
- 2026-08-29：DG27-03 关闭。current nominal 上体复合参数明确剔除两轮，姿态复用
  Phase23 relative rotation-vector chart，Eq.(12) 使用已关闭的 internal wrench；单个
  20 ms RK4 的 v1 one-step gate FAIL 被保留，未放宽阈值，改为同一 ZOH 内两个固定
  10 ms RK4 substep 后 v2 oracle 全 PASS，C++ AutoDiff parity 与 Core 29-test suite PASS。
- 2026-08-29：DG27-04 关闭并保留 `2/10/20 ms`。ad-hoc 强扰动 timing-v1 两个
  profile 都失去接触且 evaluator 文案错误，完整保留为 FAIL；v2 改用 Phase21 已冻结
  combined-positive case，两个 profile 的 hold/disturbance/replay、contact 和 deadline
  gate 均 PASS。`1/5/20` 未关闭任何失败 gate 却使 physics/WBC 负载翻倍，因此不批准。
- 2026-08-29：T06 关闭。16-state v2 checked-in artifact 的 clean 双生成与 normalized
  hash、generated next/A/B、equilibrium/正负参考/brake/return/wheel common+differential、
  bounds/defect/projected-stationarity/reset/10 ms deadline 全 PASS；全包 30 tests PASS。
  v1 projected-stationarity 计算漏掉 acados `EULER` running-cost 的 `Ts` 缩放，故保留
  但撤回为 inconclusive，不把错误 audit 当成模型或 solver FAIL，也没有放宽门槛。
- 2026-08-29：T07 关闭。opt-in Minimal profile 的 42D order 与 104 hard rows/bounds
  对 nominal 逐元素不变；独立 Hessian/gradient、interaction affine+bias、signed slack、
  disabled-task invariance、4-state dynamic corpus、torque extraction、reset 与 10 ms deadline
  全 PASS。全包回归增至 31 tests，零 error/failure；尚未作 runtime/plant claim。
- 2026-08-29：T08 关闭。Phase 27 opt-in runtime、2/10/20 phase/ZOH/age、四类
  NMPC fault 的 exact-zero latch/reset、additive log schema 与 runner non-overwrite
  通过 component/smoke 验证；Release suite 为 32/32。100-tick 非 formal hold smoke
  在 tick 58 越过既有 x safety envelope，此结果保留给 T10 的预冻结 T0 判定，不在
  T08 添加 state task 或改变 safety gate。T09/DG27-05 转执行。
- 2026-08-29：T09～T11 关闭。DG27-05 在 primary 前冻结 6-case T0～T3 config、
  thresholds、failure enum 与 synthetic evaluator oracle。formal-v1 因 evaluator 的
  moving-reference tick 对齐和 latch-zero解释错误保留为 inconclusive；不改 controller/
  threshold 的 append-only v2 判定 Minimal FAIL：T0 为 0.58 s safety envelope，T1/T2
  均为 0.45 s safety envelope，T3 `+/-10 mm` 为 0.04/0.08 s native NMPC stationarity
  audit。fresh replay、四类 exact-zero fault/reset、non-overwrite 与最终 32/32 Release
  regression PASS。REVIEW=PASS、blocking finding=0，Phase 按允许的 diagnosed FAIL
  terminal outcome 完成；不在本 Phase add-back 或 retune。

## Blockers

- DG27-02 是 16-state model、Eq.(12) 与 Minimal WBC 实现的共同硬前置；无法证明
  wheel-center internal interaction-wrench closure 时，本 Phase 必须 REWORK，不能退回
  “直接使用 external contact wrench”而仍宣称理论恢复。
