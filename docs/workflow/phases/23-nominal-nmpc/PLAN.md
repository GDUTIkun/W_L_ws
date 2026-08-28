# Phase 23: nominal NMPC — PLAN

Status: `blocked`

## Goal

在不连接真机、不改变 Phase 21/22 Weighted WBC 数学与 canonical robot I/O 的前提下，为 current nominal full-3D MuJoCo profile 建立 runtime-independent C++ nominal NMPC：从 canonical `RobotState` 和内部确定性 reference 生成左右 12D interaction-wrench command，经冻结的 ProxQP-backed WBC 输出六路力矩，并以模型/oracle、OCP/KKT、组合时延、完整 MuJoCo formal、fault/reset、fresh replay 和非覆盖证据证明该新增上层在声明范围内可用。

## Current State

- 已有：[Phase 21](../21-nominal-weighted-wbc/RECORD.md) 已冻结 current nominal 12-DoF、42-variable/104-row Weighted WBC、内部 `WbcReference.interaction_wrench_flu` 12D 接口、10 ms Core 周期、fail-zero/latch/reset 及 19 normal/perturbation + 6 fault formal；[Phase 22](../22-proxqp-solver-migration/RECORD.md) 已在不改变上述数学的前提下把 QP backend 迁移为 ProxQP v0.7.3，并完成 component/oracle/deadline/formal-v2/fresh replay。
- 已有：live `ControllerCore::stepWeightedWbc` 当前从 `RobotState` 内部生成 standing acceleration/reference 和固定 nominal interaction wrench，再调用 `WeightedWbcController`；production C++ 尚无 NMPC。最小接入点是只替换 12D wrench producer，保留其他 WBC reference、solver、torque extraction 和安全边界。
- 已有：Simulink baseline 的 `full_base_nmpc_state_signal.m`、`full_base_body_dynamics.m`、`full_base_nmpc_ocp.m` 和 `full_base_nmpc_reference.m` 提供 16-state/12-input、20 ms、N=20、0.4 s、SQP-RTI/acados 历史候选及参考实现。
- 历史限制：该候选使用 Euler state、Simulink/acados 生成 S-function、历史数值模型/weights/bounds，并由 `full_base_nmpc_command.m` 在失败时保持 last-valid command；这些均未获 current production 批准，且 last-valid-hold 与当前 fail-zero/latch 语义冲突。
- Grounding：CBM project `W_L_ws`、generation `2026-08-28T07:49:08Z`、full index。Core/WBC 六个 live C++ 路径和六个 Simulink NMPC 路径均为 `no_recorded_issue + metadata_match`；该 coverage 仅为 best-effort。`stepWeightedWbc` 的关键入口由 live source/高置信关系支持，部分 heuristic 边不作为唯一依据。Graphify 只确认 NMPC→wrench→WBC 的历史路线和候选关系，不把历史图当作当前源码或批准证据。

## Scope

- 冻结 12D nominal base NMPC state、12D contact-wrench input、reference、frame/order/sign/unit、离散时序、状态有效域和 stale/reset 语义，并建立 canonical `RobotState -> NmpcState` 映射；16D历史候选的`xi/dxi`只保留为workspace诊断，不进入OCP。
- 从 current nominal Phase 21 model/profile 重新推导 runtime-independent C++ nominal dynamics、RK4 离散模型和解析一阶 sensitivity；以独立 Python/MuJoCo 数值 oracle 与 Simulink 候选逐层核对，冲突以 current canonical contract 和真实 oracle 为准。
- 建立 versioned project-owned NMPC problem/result 边界；初始 OCP candidate 为 20 ms、N=20、0.4 s、single SQP-RTI、Gauss-Newton、multiple shooting，并以现有 ProxSuite 的 sparse ProxQP 解线性化 QP。production 实现前必须用 corpus 关闭数学、conditioning、warm/reset、determinism 和组合时延 gate。
- 只让 NMPC 生成 `WbcReference.interaction_wrench_flu`；base/height/orientation/leg acceleration reference、42D/104-row WBC、dense ProxQP adapter、hard acceptance、torque extraction 和 canonical `RobotState -> TorqueCommand` 保持冻结。
- 使用内部 versioned deterministic reference profile 验证 equilibrium hold、正/负小幅直线 longitudinal reference、回零和扰动恢复；不新增公共 ROS command/schema，不把轨迹规划、转弯或大姿态纳入本 Phase。
- 增加 opt-in NMPC+WBC Core mode、additive diagnostics、component/oracle/benchmark、独立 MuJoCo loop 日志和 fault injection；复用 Phase 22 runner/formal/evaluator/manifest 结构，不另建并行验证框架。
- 在正式输入冻结后执行 Phase 22 的 19+6 最低回归矩阵，并增加 4 个 NMPC-specific normal/reference case 与 4 个 NMPC-specific solver/late/stale/non-finite fault case；执行 fresh replay、non-overwrite、hash 审计和 Phase 14/15/18/20/21/22 compatibility regression。

## Out of Scope

- 真机、STM32/树莓派、Hardware Adapter、正式通信协议、传感器/执行器/contact 辨识、目标硬件时延和任何 real/identified-profile 结论。
- trajectory/foothold planner、terrain、斜坡/台阶、单轮支撑、跳跃、跌倒恢复、roll/yaw recovery、yaw-rate/turning、continuous turning 或 large-yaw；这些保留给后续 Phase。
- 改变 Phase 21 的 12-DoF model、42D decision order、104 hard rows、task/weight/scale、wrench/slack sign、WBC 10 ms timing、dense ProxQP adapter、torque limit 或 fail-zero/latch/reset。
- 修改 canonical FLU、quaternion/world twist、joint order/sign/unit、公共 `RobotState/TorqueCommand`、ROS messages、Adapter、MuJoCo plant/contact/timestep 或 Phase 20 equilibrium。
- production 链接 MuJoCo、MATLAB、Simulink、CasADi 或 acados；迁入/生成 acados S-function/C code；把历史 Euler state、weights/bounds、last-valid-hold 或 solver PASS 直接复制为 production authority。
- 在线辨识、自动调权、observer、积分器、gain scheduling、异步线程/队列或新的外部依赖；若同步 candidate 无法关闭 deadline gate，先 REWORK 本 PLAN，而不是顺带引入并发架构。
- 覆盖 Phase 21/22 config、manifest、formal 或 evidence；仅凭编译、QP status、短时轨迹或 WBC slack 中任一单项宣称 NMPC PASS。

## Frozen Decisions

- **Phase/claim authority：** Phase 23 是 Phase 22 之后唯一新增的 nominal upper layer；结论只限 current nominal full-3D simulation reference host。Phase 21/22 的 WBC 数学和 solver evidence 是冻结下游基线，不自动证明 NMPC。
- **Production ownership：** NMPC model、linearization、OCP assembly、warm/reset 和 acceptance 由 project-owned C++17 Core 模块实现；runtime 不依赖 MuJoCo/MATLAB/Simulink/CasADi/acados，也不使用生成控制代码。历史 acados 只作独立 oracle/reference。
- **Physical state contract：** 经DG23-01 pre-freeze REWORK，物理 state 固定为 12D `x=[p_B^N(3), r_B^N(3), v_B^N(3), omega_B^N(3)]`。`r_B^N` 是 current reset/reference chart 内、与 Phase 21 shortest-arc orientation error 同号同轴的 world-axis rotation vector，不是 Euler angle；姿态传播用 quaternion/Exp-Log 后再投回冻结的小姿态 chart。Phase 15/21 `xi/dxi`映射仍在每拍作为轮/闭链workspace诊断，但因历史internal-wrench端口与current external-contact-wrench端口不等价而不进入production OCP。
- **Input contract：** `u=[W_left_FLU(6), W_right_FLU(6)]`，每侧顺序 `[Fx,Fy,Fz,Tx,Ty,Tz]`、单位 N/N·m，与 `WbcReference.interaction_wrench_flu` 完全一致。NMPC output 只进入该字段；不得通过 ROS、MuJoCo truth 或旁路直接输出 torque。
- **Model authority：** dynamics 必须从 current nominal locked-composite mass/COM/inertia/gravity、canonical base-control frame重新推导，并逐点对current 12-DoF reduced mass/bias核对。历史 `full_base_body_dynamics` 仅作对照；其wheel-to-body internal wrench、rolling denominator和lever-arm重建不得用于current已运输的external contact-wrench输入。
- **OCP candidate：** 首个待证候选固定 `Ts=0.020 s`、`N=20`、horizon `0.40 s`、RK4、Gauss-Newton、one SQP-RTI step、multiple shooting；physical state 为经DG23-01修订的12D locked-composite base state，delta-input cost 以 previous-applied wrench 参数锚定。若 DG23-02 不通过，必须先记录证据并修订 PLAN，不能在 production 中静默切换 horizon、solver family 或 fallback。
- **QP candidate：** NMPC linearized QP 使用独立 project-owned adapter 调用 `proxsuite::proxqp::sparse::QP<double>`；不扩大或复用 WBC 的 42-variable/128-capacity `DenseQpSolver`。backend、scaling、regularization、termination、warm compatibility、status 和 residual schema 由 DG23-02 corpus 冻结；无旧算法或 dense/sparse 自动 fallback。
- **Constraint ownership：** NMPC input/contact-wrench constraints从 Phase 21 validated wrench/contact-frame contract映射，state/workspace bounds不得弱于 Phase 15/21 safety envelope；WBC 的 dynamics、torque、H-cone、acceleration和candidate audit仍是最终 actuator-level authority。NMPC 约束不可替代 WBC hard gate。
- **Reference boundary：** Phase 23 只使用内部、versioned、确定性的 equilibrium 与小幅直线 longitudinal reference；Y、roll/pitch/yaw nominal reference保持 reset equilibrium，禁止 turning/large-yaw。reference amplitude、rate、tuning/holdout split和tracking gate必须在 DG23-03 holdout 前一次性冻结。
- **Schedule：** physics `0.002 s`、WBC `0.010 s` 保持不变；NMPC candidate 每两个 WBC tick 同步更新一次并在中间 tick 做两拍 ZOH。更新 tick 的 `NMPC step + WBC step` 在 reference host 上必须小于 `10 ms`；禁止用20 ms supervisor period掩盖阻塞 WBC deadline。reset 后首解 cold，replay 的 update/ZOH phase 必须精确一致。
- **Safety/staleness：** 只有本拍成功、finite、通过独立 OCP/constraint audit且 age 在两拍 ZOH 合同内的 wrench 可交给 WBC。timeout、non-finite、infeasible/unbounded、iteration limit、KKT/hard gate失败、sequence/timestamp错误或 stale 均使本拍六路 torque 严格为零并锁存到 reset；不无限保持 last-valid，不使用 nominal wrench 或旧 standing mode作 fallback。
- **Compatibility：** `kZero`、Phase 17/19/20 modes和 `kWeightedWbc` 行为保持不变；NMPC 以新 opt-in mode/additive diagnostics进入。public Adapter/watchdog/ROS conversions不变，WBC non-NMPC component corpus必须逐项回归。
- **Evidence authority：** Phase 23 config/method/result使用新 namespace、source/config/output hash与不存在或空的run目录。primary/replay除明确允许的 wall-clock 字段外确定性一致；失败后新建run并记录 `supersedes`，不得覆盖旧 evidence。

## Open Questions / Decision Gates

- **DG23-00 / CLOSED / CODEX — route：** 经DG23-01 pre-freeze修订，采用 nominal 12-state locked-composite base/12-wrench upper NMPC，只替换 WBC interaction-wrench producer；不合并 trajectory/turning/terrain/real work。原16-state历史candidate仅作被拒绝的对照与xi workspace诊断。
- **DG23-01 / OPEN / CODEX+EVIDENCE — state/model：** canonical state chart、RobotState/xi mapping、continuous dynamics、RK4、analytic sensitivity、validity envelope和equilibrium必须通过 independent oracle、finite difference、virtual-work/energy/sign/order检查；通过前不得实现 production solver/Core。
- **DG23-02 / OPEN / CODEX+EVIDENCE — OCP/solver/timing：** candidate OCP、sparse ProxQP profile、conditioning、KKT、warm/reset、SQP-RTI contraction、1000-run determinism和更新 tick组合 `10 ms` deadline必须在离线 corpus 上关闭；失败则 REWORK candidate，不引入 silent fallback。
- **DG23-03 / OPEN / CODEX+EVIDENCE — reference/cost/constraints：** stage/terminal cost、normalization、wrench/delta-wrench weight、input/state constraints、reference amplitude/rate、tuning/holdout split和正式 tracking/recovery thresholds必须以独立 oracle、ablation、tuning和未见 holdout 关闭；历史数值仅作起点。
- **DG23-04 / OPEN / CODE+TEST — runtime contract：** 新 opt-in mode、2:1 schedule、ZOH、timestamp/age、diagnostics、NMPC→WBC order、fault zero/latch/reset、旧mode与 WBC component回归必须通过。
- **DG23-05 / OPEN / EVIDENCE — integrated formal：** frozen 23 normal/reference + 10 fault matrix、solver/model/WBC/plant gates、fresh replay、non-overwrite和Phase14/15/18/20/21/22兼容性必须全部通过。
- **DG23-06 / OPEN / REVIEW — claims：** blocking findings为零且 REVIEW=`PASS` 后才可创建 RECORD、将 ROADMAP 标记 complete 并放行后续 roll/yaw/turning。

## Interfaces and Compatibility

- 输入：canonical `RobotState`；内部 versioned `NmpcReference`；Phase 21 current nominal model/equilibrium/contact-wrench profile；20 ms NMPC phase与10 ms WBC control tick。
- 内部：`RobotState -> NmpcState12 -> NmpcProblem -> NmpcResult{wrench12,status,KKT,age}`；accepted wrench只写入现有 `WbcReference.interaction_wrench_flu`，随后走冻结 `WeightedWbcController`。
- 输出：canonical six-channel `TorqueCommand`；additive NMPC diagnostics；model/OCP/solver benchmark、control/plant CSV、summary、manifest和append-only evidence。
- 必须保持：Phase 15 coordinate/workspace、Phase 21 WBC model/QP/task/reference其余字段、Phase 22 dense solver、FLU/quaternion/world twist、joint order/sign/unit、2/10 ms timing、5-step torque ZOH、fault latch/reset、public messages、Adapter/plant和既有 evidence。
- 允许改变：`wheel_leg_core`新增独立 nominal NMPC model/problem/solver/controller模块、config/result/diagnostics/tests，并在 `ControllerCore` 增加 opt-in NMPC+WBC mode；`wheel_leg_mujoco`新增或最小扩展独立 loop target；新增 Phase 23 oracle/benchmark/wrapper/config/method和 evidence。任何超出此列表的改动先修订 PLAN。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| P23-T01 | 固化grounding、provenance与禁止复制清单 | Phase15/20/21/22、live Core/WBC、Simulink NMPC、CBM/Graphify | source/graph/coverage记录、current-vs-history差异、最小影响面与复用入口 | 候选源码coverage与fallback完整；历史candidate不被当作production authority | done |
| P23-T02 | 冻结12D state/input/reference/time contract并审计16D历史candidate | P23-T01、RobotState、Phase15 xi、WbcReference | exact order/frame/sign/unit/chart、mapping、validity/stale/reset spec、golden vectors与model-closure decision | DG23-01 mapping部分；finite-difference/边界/fault tests PASS，internal/external wrench冲突已关闭 | done |
| P23-T03 | 建立independent model与sensitivity oracle | P23-T02、current nominal profile、历史body dynamics | continuous/RK4 model、analytic Jacobian、equilibrium/energy/virtual-work/sign oracle、versioned corpus | DG23-01关闭；model/sensitivity误差和有效域满足冻结门槛 | done |
| P23-T04 | 冻结OCP数学与sparse ProxQP profile | P23-T03、candidate Ts/N/RTI、Phase22 dependency | exact decision/constraint/cost/scaling spec、solver adapter prototype、golden/failure corpus与1000-run benchmark | DG23-02；KKT、oracle、warm/reset、determinism、contraction、组合deadline PASS | blocked |
| P23-T05 | 冻结reference/cost/constraint profile | P23-T04、Phase22 envelope、预声明tuning/holdout cases | versioned weights/bounds/reference、ablation/attribution、tracking/recovery/fault thresholds | DG23-03；tuning后冻结，未见holdout与nonlinear pre-freeze全部PASS | todo |
| P23-T06 | 实现runtime-independent C++ NMPC组件 | P23-T02～T05冻结spec | state mapper、model/sensitivity、OCP assembly、sparse solver、warm/reset/result diagnostics | 与golden逐项一致；production library无oracle依赖；component/failure tests PASS | todo |
| P23-T07 | 集成additive NMPC+WBC Core mode | P23-T06、现有WbcReference/Core safety | opt-in mode、2:1 update/ZOH、wrench injection、status/age/KKT diagnostics、zero/latch/reset | DG23-04 Core部分；NMPC→WBC顺序、stale/late/failure和旧mode回归 PASS | todo |
| P23-T08 | 建立/扩展full-3D NMPC loop与日志 | P23-T07、Phase22 runner/Adapter | 最小loop target、deterministic references/faults、逐tick NMPC/WBC/control/plant日志 | DG23-04关闭；2/10/20 ms相位、5-step torque ZOH、双时钟、truth隔离和replay PASS | todo |
| P23-T09 | 建立正式方法、profiles与evaluator | P23-T05/T08、Phase22 formal schema | `docs/experiments/`方法、versioned model/solver/reference/formal config、wrapper/evaluator/manifest schema | formal前freeze；依赖探针/py_compile/non-empty拒绝/hash/schema/case/threshold完整 | todo |
| P23-T10 | 执行full formal、fresh replay与历史回归 | P23-T09 frozen inputs、current nominal plant | 新`evidence/automated/<run-id>/`、summary/manifest/replay/non-overwrite/regression audit | DG23-05；23 normal/reference、10 fault、model/OCP/WBC/plant/replay/history全部PASS | todo |
| P23-T11 | REVIEW | 全部任务、live source和真实evidence | `REVIEW.md`；仅PASS后创建`RECORD.md` | DG23-06关闭、blocking findings=0后才更新ROADMAP complete | todo |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Pre-freeze Model and OCP

- `RobotState -> x12` golden 覆盖 equilibrium、Phase15 workspace、正负小姿态/速度和全部 invalid boundary；位置/速度/姿态映射与独立实现最大误差 `<=1e-9`，超出冻结chart/workspace必须拒绝。`xi/dxi`仅作附加workspace oracle，不作为OCP state。
- continuous dynamics与RK4 next-state在 equilibrium、对称、正负 wrench、workspace和dynamic corpus对 independent double-precision oracle最大绝对误差 `<=1e-8`；解析 Jacobian 对中心有限差分最大 scaled error `<=1e-5`。finite-difference只用于测试，不进入 runtime。
- equilibrium 导数、left/right symmetry、gravity/gyro、frame transform、virtual work、input sign/order和 rollout consistency逐项判定；Simulink候选差异必须解释，不能用轨迹吻合替代局部数学检查。
- 对每个线性化 QP 独立重算 objective、equality/inequality violation、stationarity和complementarity；accepted candidate 最大 primal/stationarity `<=2e-7`，首个 wrench 与 independent oracle差 `<=5e-4 N/N·m`，objective gap `<=2e-6`。vendor status/residual不单独授权输出。
- cold、repeated-warm、cycling dynamic warm各1000次；记录linearization、assembly、solve、total、iteration、allocation、host/compiler和完整settings。NMPC-only及更新tick `NMPC+WBC` total均须 `<10 ms`，且 reset后首解与cold deterministic。

### ROS Build and Component Tests

```bash
cd /home/t/W_L_ws/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to wheel_leg_mujoco \
  --cmake-clean-cache --cmake-args -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=Release
colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco
colcon test-result --verbose
```

- 所有既有Core/ROS/MuJoCo tests和新增state/model/OCP/solver/Core/runner tests为0 failures；warnings-as-errors保持通过。
- `kWeightedWbc` 对冻结 Phase 22 component corpus逐项不变；新mode所有非成功、late/stale/non-finite路径从故障拍起六路严格zero并锁存，reset恢复cold和相位零点。

### Formal MuJoCo

在创建稳定输出目录前使用仓库解释器完成依赖探针和语法检查：

```bash
cd /home/t/W_L_ws
./.venv/bin/python -c "import mujoco, numpy, scipy; print(mujoco.__version__, numpy.__version__, scipy.__version__)"
./.venv/bin/python -m py_compile <phase23-oracle-wrapper-evaluator-files>
./.venv/bin/python <phase23-formal-wrapper> \
  --output-dir docs/workflow/phases/23-nominal-nmpc/evidence/automated/<new-run-id>
```

- 正式矩阵最低为继承 Phase 22 的19 normal/perturbation + 6 fault，并追加4个预冻结 NMPC straight-reference/return/recovery normal cases和4个 solver-error/late/stale/non-finite fault cases；所有 case 使用同一 frozen model/solver/reference profile。
- Phase 22 plant/contact/slip/closure/WBC hard/task/torque/deadline gate不得变弱；追加model/OCP/KKT、reference tracking/recovery、NMPC update/ZOH phase和age gate。tracking具体幅值与阈值由P23-T05在holdout前冻结并写入versioned config。
- 输出目录必须不存在或为空；primary/fresh replay除明确列出的wall-clock字段外逐字段一致，plant CSV字节一致；manifest记录解释器/依赖、model/solver/reference/controller/runner/scene/config/output hash。
- fresh执行Phase14/15/18/20回归，重跑Phase21/22 frozen WBC component/formal兼容性入口；旧config/manifest/evidence hash保持不变。

## Acceptance Criteria

- [x] DG23-01关闭：12D base state/chart、12D external contact wrench、canonical mapping、locked-composite continuous/RK4 model、analytic sensitivity、equilibrium和有效域通过独立oracle；历史Euler/acados/internal-wrench/16D candidate差异已解释。
- [ ] DG23-02关闭：20 ms/N=20 candidate或经REWORK明确批准的替代方案已冻结；sparse ProxQP QP满足KKT/oracle/warm/reset/determinism和1000-run更新tick组合 `<10 ms` 门槛，无silent fallback。
- [ ] DG23-03关闭：cost/scale/constraints/reference/tuning-holdout split与正式tracking/recovery thresholds在holdout前冻结，ablation、attribution和未见 nonlinear holdout 全PASS。
- [ ] production C++不依赖MuJoCo/MATLAB/Simulink/CasADi/acados或生成控制代码；NMPC只写现有12D wrench boundary，WBC/torque/public I/O保持冻结。
- [ ] component/build tests全PASS；2:1 schedule、两拍wrench ZOH、timestamp/age、warm/cold/reset和所有failure路径保持deterministic fail-zero/latch。
- [ ] formal完成23 normal/reference + 10 fault，model/OCP/WBC/plant/deadline gates全PASS；fresh replay、non-overwrite/hash和Phase14/15/18/20/21/22兼容性回归全PASS。
- [ ] Phase21/22 source-of-truth config、manifest和evidence未被覆盖；Phase23所有结论引用新namespace和真实hash。
- [ ] REVIEW blocking findings为零且Verdict=`PASS`后才创建RECORD、把ROADMAP标记complete并开始roll/yaw/turning后续Phase。

## Execution Notes

按任务 ID 在本文件记录实际命令、结果、偏差和证据链接；不要建立第二份任务状态表。P23-T01～T05与DG23-01～03关闭前不得实现production NMPC/Core。任何需要改变state/input、public I/O、WBC数学、solver family、20 ms schedule、同步执行或plant的发现都先保留失败证据并将本Phase置为REWORK/blocked后修订PLAN；formal失败不得通过放宽Phase22 plant/WBC gate、隐藏fallback或覆盖旧run修复。

- 2026-08-28：P23-T01完成live CBM Verify、Graphify历史查询、20个精确路径与两个production scope coverage、三个parse-partial range源码fallback及clean Release基线。确认production无NMPC，最小接缝仅为现有12D `WbcReference.interaction_wrench_flu`；历史Euler/acados/last-valid与current contract冲突，不获继承。四packages构建通过，ROS汇总`24 tests, 0 errors, 0 failures`。详见[evidence/grounding.md](evidence/grounding.md)。P23-T02进入doing，production仍未修改。
- 2026-08-28：P23-T02冻结`base_control_frame`原点、spatial shortest-arc rotation-vector、world twist及含旋转项的轮心相对坐标；12D input明确为已运输到同一base-control点的FLU wrench，禁止二次加入接触lever arm。依赖探针为MuJoCo 3.7.0/NumPy 2.2.6/SciPy 1.15.3。append-only `state-oracle-v1`因evaluator把静态xi误计为speed而保留为superseded FAIL；修正后的`state-oracle-v2`九门全PASS，最大映射FD误差`1.38e-10`、rotation-rate误差`8.01e-11`、determinism为0。详见[evidence/state-contract.md](evidence/state-contract.md)。P23-T03进入doing，DG23-01尚未关闭。
- 2026-08-28：P23-T03 pre-freeze closure审计拒绝历史16D production candidate：历史输入是wheel-to-body内部wrench，current WBC reference是已运输到base-control点的external contact wrench；复制历史xï与moment方程会混淆物理端口并二次计算lever arm。按DG23-01修订为12D locked-composite base candidate，xi/dxi仅保留workspace诊断；公共12D wrench/WBC边界、20ms/N=20候选和安全语义不变。详见[evidence/model-closure-decision.md](evidence/model-closure-decision.md)。
- 2026-08-28：P23-T03 authority `model-oracle-v5`十门全PASS：current reduced dynamics误差`1.18e-11`、equilibrium导数`3.42e-15`、RK4-vs-DOP853 `1.06e-9`，并新增非零yaw-anchor验证`R=Exp(r)R_ref`。C++ AutoDiff golden test的continuous/RK4误差`3.11e-15/5.56e-17`、continuous/discrete sensitivity误差`1.34e-9/4.13e-11`；invalid/chart fail-closed，Release Core 7/7且repo summary 25/25 tests。DG23-01关闭，详见[evidence/model-oracle.md](evidence/model-oracle.md)。P23-T04进入doing。
- 2026-08-28：按用户指令立即冻结Phase 23。未编译验证的sparse-OCP prototype及其CMake target已撤回；P23-T01～T03和DG23-01的真实PASS证据保留。P23-T04标记blocked，P23-T05～T11保持todo；未进入REVIEW、未创建RECORD、未宣称NMPC可用。恢复时从P23-T04 OCP/solver pre-freeze继续。

## Blockers

User-directed freeze on 2026-08-28. Phase 23停在P23-T04之前；解除冻结前不得继续OCP/solver、production Core、MuJoCo formal或claims。恢复入口为已关闭DG23-01之后的P23-T04，不能从模型PASS推断NMPC PASS。
