# Phase 21: nominal Weighted WBC — PLAN

Status: `active`

## Goal

在不连接真机、不引入 NMPC 的前提下，为 current nominal 完整 3D MuJoCo plant 建立一个运行时不依赖 MuJoCo 的 12-DoF reduced-model Weighted WBC：以刚体动力学、接触/摩擦和六路力矩边界为硬约束，以站立运动、软接触、interaction-wrench fidelity 与正则项为归一化加权任务，经 canonical `RobotState -> TorqueCommand` 闭环完成 Phase 20 同等级小扰动站立、fault/reset、replay 和非覆盖验证，并保留 Phase 22 可直接接入 NMPC wrench command 的内部接口。

## Current State

- 已有：[Phase 14](../14-mujoco-internal-dynamics-validation/RECORD.md) 已验证 current nominal MuJoCo 的 `M(q)`、重力、正逆动力学、闭链、能量和开环回放自洽；完整 plant 的 unconstrained velocity dimension 为 16，floating 双闭链约束有效降维后应形成 12 个独立速度方向，但该 12-DoF reduced dynamics 尚未形成 production C++ 模型。
- 已有：[Phase 15](../15-mujoco-closed-chain-kinematics/RECORD.md) 已验证每侧 `[hip,knee,wheel]` 独立坐标、两被动关节装配解、constraint reduction、轮心/接触点 reduced Jacobian、有限差分、速度和虚功；其 Python/MuJoCo oracle 可作为 Phase 21 独立对照，不能直接成为 runtime controller。
- 已有：[Phase 18](../18-mujoco-contact-floating-base-plant-validation/RECORD.md) 已验证 wheel-only contact、normal/rolling/lateral/friction、floating reset/touchdown；contact force、slip 和 penetration 仍是 validation-only plant truth，不在 `RobotState` 中。
- 已有：[Phase 20](../20-nominal-3d-simple-standing/RECORD.md) 已给出 full-3D equilibrium、10 ms controller/2 ms physics/5-step ZOH、19 个 10 s normal/perturbation、6 个 fault cases、plant/contact/slip/closure 和 replay authority，可作为 WBC formal 的最低 case/envelope 基线；其静态 `u=-Kx`、support torque 和 gain 不是 WBC 模型或权重。
- 已有：Simulink 对照基线的 `spatial_two_leg_qp_core.m` 定义 12-DoF、36-variable WM-WBC，含 hard dynamics、torque/friction bounds、normalized weighted tasks、interaction-wrench slack 和 KKT/`quadprog` 求解路径；其 controller/physical frame permutation、Euler state、Simscape workspace、legacy contact task和 5 ms timing不能直接复制到当前 C++/MuJoCo 路径。
- 当前生产侧：`ControllerCore::step(const RobotState&) -> StepResult`、完整 base pose/twist、六 active joint state、左右 contact enum 和 canonical six-torque boundary 已存在；Core 目前没有 C++ rigid-body dynamics、passive-joint reconstruction、linear-algebra/QP solver、contact-force接口或 WBC mode。
- Grounding：CBM project `W_L_ws`、generation `2026-08-26T07:29:33Z`；Phase 20 后 Core 源码为 `metadata_changed`、`standing_3d_loop.cpp` 未跟踪于该 generation，已按 coverage 建议直接读取 live source。`docs/`、`tools/` 不在 CBM 主索引，历史设计与 Phase 关系由现有 Graphify 图和真实文档复核。

## Scope

- 以 `simulation/mujoco/model/phase18_floating_contact.xml` 及其 authoritative `wheel_leg.xml` 为唯一 plant，保持 full freejoint、双闭链、六 actuator、wheel-only contact、solver/contact/timestep 和 Phase 20 equilibrium，不派生 planar plant、不增加 hidden weld/辅助外力。
- 冻结 12-DoF reduced state/model contract：base translation、base orientation tangent、左右各三 active coordinates；每侧 passive coordinates/velocities由冻结装配分支和 constraint reduction从 canonical active state重构，禁止 Controller 从 MuJoCo private state读取被动关节。
- 建立 runtime-independent nominal model profile和 C++ model boundary，输出 reduced mass matrix、bias、actuation map、左右 contact Jacobian/bias、interaction-wrench map及重构诊断；使用独立 MuJoCo/Python oracle逐层验证。
- 保留历史36-variable Weighted WBC合同与证据用于回归，但其single-force contact representation已失去当前authority；新的decision vector、contact变量维度与hard-QP只能在连续contact representation通过local oracle后重新冻结。
- 定义 Controller 内部、非 ROS 的 versioned `WbcReference/WbcProblem/WbcSolution` 或语义等价边界。Phase 21 的 reference producer只生成 nominal standing target/equilibrium interaction wrench；Phase 22 可替换 producer接入 NMPC，但不得改变本 Phase冻结的 WBC sign/order/slack语义。
- 审核并冻结一个 C++17 CPU solver路径，验证 convexity、conditioning、warm start、active bounds、确定性、deadline和失败语义；任何不可验证或违反 hard constraints 的 fallback都不得输出非零力矩。
- 在 Core 修改前完成 model oracle、solver component、equilibrium QP、逐层约束/任务和 nonlinear pre-freeze；失败时保留证据并 REVIEW=`REWORK`，不通过放宽约束、隐藏外力或复制 Phase 20 torque继续实现。
- 新增独立 opt-in WBC mode、additive diagnostics、单元/性质测试和独立 full-3D WBC runner；复用 canonical Adapter、watchdog、双时钟、2 ms/10 ms/5-step ZOH、fault/reset/replay和non-overwrite。
- 建立正式实验方法、versioned model/solver/task/formal profiles、layered case matrix、逐 control-tick solver/task/constraint日志、plant truth日志、summary/manifest/hash，并执行 fresh replay、历史回归和 revision reuse dry-run。

## Out of Scope

- 真机、STM32/树莓派、Hardware Adapter、传感器/执行器/contact辨识，以及任何 MuJoCo-real、安全或实时部署结论。
- NMPC/OCP、trajectory planning、上层速度/转向命令生成、Simulink Acados solver迁移、terrain adaptation、斜坡/台阶、单轮支撑、跳跃或跌倒恢复。
- 修改 canonical FLU、joint order/sign/unit、公共 `RobotState/TorqueCommand` 或 ROS message schema；不把 contact force、MuJoCo `qfrc_*`、passive qpos/qvel或 mass matrix塞入公共消息。
- runtime 链接 MuJoCo、从 runner 旁路读取 plant truth、有限差分在线线性化、在线参数辨识、自动调权、积分器、observer或 gain scheduling。
- 把 Simulink 的 46D input、198D diagnostics、5 ms timing、`quadprog` fallback、数值 weights/torque/contact参数整体照搬；把 `QP feasible`、小 slack、无饱和或短时稳定中的任一单项写成 WBC PASS。
- 修改 Phase 20 simple-standing controller或覆盖 Phase 14/15/18/20 的正式 evidence；WBC不要求替代旧 mode作为默认控制器。

## Frozen Decisions

- **Plant/claim authority：** 仅 current nominal full-3D MuJoCo simulation；所有 model/contact/solver/task/formal evidence使用新run目录和hash。identified/new CAD profile必须重新经过本Phase全部model、QP和formal gates。
- **Runtime separation：** Controller Core和WBC production library不得链接、调用或持有 `MjModel/MjData`；MuJoCo只作为独立oracle与plant。runner记录的 contact force/slip/closure不得反馈给Controller。
- **Canonical reduced coordinates：** configuration由`(p_B^N,q_N_from_B,q_active6)`表示；velocity固定为`nu=[v_B^N,omega_B^N,dq_active6]`，其中两类base velocity均为world-axis语义，QP变量前12维为与该tangent一致的`nudot`。orientation error继续使用Phase20 world-axis shortest-arc Log，不改为Simulink Euler state。
- **Closed-chain reconstruction：** passive state由Phase15冻结branch、closure equation和velocity reduction重构；branch ambiguity、closure/conditioning超限、非有限或超出冻结workspace均fail closed。若 canonical `RobotState` 无法在目标精度/deadline内可靠重构，Phase必须REWORK，不隐式扩张公共schema。
- **Contact/force order：** 每轮lambda顺序冻结为`[rolling,lateral,normal]`，left block在right block前，positive normal指world `+Z`支撑，rolling positive与Phase15 canonical `+X`无滑方向一致。interaction wrench每侧固定为controller FLU `[Fx,Fy,Fz,Tx,Ty,Tz]`，left block在right block前。
- **Continuous contact candidate：** current-nominal local-oracle候选冻结为解析wheel-ground frame、两bottom lateral endpoints加四个1 mm band-edge corners的连续六点surface patch；point force在contact frame按`[rolling,lateral,normal]`定义。force support不生成六个rigid constraints；soft contact-acceleration候选仍是每轮contact center处三行`[rolling,lateral,normal]`。
- **Decision vector/slack：** 当前42D order冻结为`z=[nudot_12,tau_6,w_left_C6,w_right_C6,slack_left_FLU6,slack_right_FLU6]`。contact-centred wrench每侧顺序为`[rolling,lateral,normal,rolling-moment,lateral-moment,normal-moment]`；future fidelity slack每侧为controller FLU `[Fx,Fy,Fz,Tx,Ty,Tz]`，符号固定为`W_feasible = W_reference + slack`，且不进入任何hard row。历史36D single-force order和24-row equality仅保留回归语义。
- **Hard constraints：** reduced rigid-body dynamics为12行等式；六路canonical torque box、每轮37-row contact-centred H-cone和12路componentwise acceleration box组成92行不等式，总行数固定由block counts推导为104。P21-T05确认current authoritative MuJoCo六个active joints均没有`limited/range`合同，因此本profile明确不虚构state-dependent joint-position/velocity protection；保留12D acceleration box及Phase15 reconstruction/workspace fail-closed保护。hard residual超限的candidate无论solver状态为何都不可接受。
- **Weighted tasks：** rigid contact acceleration是normalized soft task，不伪装成与Phase18 compliant contact相同；站立任务至少覆盖base height、roll/pitch/reset-heading、common rolling/base-X anchor、左右active leg posture，并将absolute Y作为outcome/safety而非独立可控task。所有residual先按物理单位scale归一化，再施加非负无量纲weight。
- **Reference boundary：** Phase21 standing producer使用Phase20 equilibrium与reset anchor生成zero-motion/nominal-wrench reference；不消费NMPC。内部12D wrench command/slack接口必须可version、可测试且不暴露为新的public robot I/O，Phase22只能替换上游producer。
- **Timing：** 沿用physics `0.002 s`、WBC control `0.010 s`、5-step ZOH；不复制Simulink 5 ms WBC周期。manifest记录model、solver、reference-host和逐tick solve time；simulation deadline PASS不推断树莓派实时性。
- **Solver safety：** warm start只能改变数值路径，不能改变数学解或reset determinism。solver error、timeout、non-finite、infeasible/unbounded、iteration limit、hard residual/margin失败均输出六路零并锁存到reset；禁止“equality solve后clip torque/lambda”作为可接受fallback。
- **Layered pre-freeze：** model/sign/oracle、solver、equilibrium、hard bounds、soft contact、motion tasks、wrench/slack和10 s nonlinear holdout依次通过后，才允许集成production Core；每层保存独立结果，不能只凭最终轨迹反推中间层正确。
- **Compatibility：** `kZero`保持默认，Phase17/19/20 modes及diagnostics行为不变；WBC通过新opt-in mode和additive diagnostics进入。公共Adapter/watchdog/fail-zero和ROS conversions不改变。
- **Formal authority：** Phase20的19个normal/perturbation和6个fault case是最低回归矩阵，plant safety gate不得更弱；Phase21另加model/QP/task/slack/deadline gates。formal输入在运行前冻结，失败后新建profile/run并记录supersedes，不原地调参覆盖。

## Open Questions / Decision Gates

- **DG21-00 / CLOSED / CODEX — route：** Phase21采用standalone nominal 12-DoF Weighted WM-WBC，保留future-NMPC wrench/slack内部接口；不把NMPC、terrain或真机纳入当前Phase。
- **DG21-01 / CLOSED / CODE+EVIDENCE — reduced model：** 12D state/passive reconstruction、continuous contact differential map和每轮三行soft Pfaffian保持冻结。closing oracle证明7个in-envelope static失败全部先出现在floating-base rows、违反同一pitch-moment support facet，same-q full-plant static oracle也全部infeasible；rolling dynamic probe `8/8` PASS。Phase15 workspace是reconstruction/kinematic envelope而非static-admissible set，7个状态均分类A（static-gate semantics）。42D hard-QP仅获准作为下一数学candidate，不继承旧36D solver/QP/Core证据。
- **DG21-02 / CLOSED / CODE+EVIDENCE — contact/wrench map：** continuous六点surface patch在analytic contact center/contact frame中形成固定`G_C:18→6`，24-ray V-cone与37-row H-cone经point/V/H三向1,240-case oracle一致，original `537/537`、fresh `540/540`及transform/virtual-work全部通过；当前冻结contract不依赖point load，允许消除每轮12D internal-force nullspace。DG21-02本身不解释static semantics或授权QP；后续授权来自DG21-01 closing Route 1。
- **DG21-03 / CLOSED / CODEX+EVIDENCE — solver：** project-owned Eigen-only fixed dense ADMM保持42变量/128-row capacity，并新增标准over-relaxation `alpha=1.6`；weighted runtime wrapper冻结`rho=0.15`。workspace-aware hard与weighted 32-case corpus均完成1000次cold、repeated-same warm与cycling-dynamic warm审计；weighted cold/dynamic最大total setup+solve=`8.273542/8.790942 ms`，最大hard/equality/stationarity=`1.128e-7/1.128e-7/4.124e-8`，最大物理力矩差`3.075e-5 N·m`。原hard solver证据保留历史，当前修复证据见`runtime_cpp_parity.md`。
- **DG21-04 / CLOSED / CODEX+EVIDENCE — equilibrium/hard feasibility：** Phase15 workspace不变；完整capture分类得到tick 1–259 in-workspace、260–271 out-of-workspace。runtime v2由全部259个eligible tick按预声明规则选28个dynamic nominal cases，tick 260/271为必须拒绝的fault cases。4个workspace与28个dynamic problems的四个累积hard layers及116-row equilibrium全部PASS，minimum cone/torque margin=`0.310102/1.99854 N·m`。原tick-271 nominal active attribution已被修复authority取代，证据见`runtime_workspace_gate_repair.md`。
- **DG21-05 / CLOSED / CODEX+EVIDENCE — task set/weights：** runtime v2重新执行42D local algebra、32-case competition accounting、4个10 s tuning和9个预冻结holdout，全部PASS且逐tick workspace failure/violation均为零。weights、gains、wrench/slack与`0.01/0.02/0.001` envelopes不变；最坏值仍为`0.003535/0.005452/0.0001502`，证据见`runtime_workspace_gate_repair.md`。
- **DG21-06 / OPEN / CODE+TEST — runtime contract：** additive WBC mode、model/solver/reference边界、diagnostics、fail-closed latch/reset、5-step ZOH、旧mode回归和无MuJoCo runtime依赖全部通过。
- **DG21-07 / OPEN / EVIDENCE — formal/reuse：** frozen formal正常/扰动/fault、solver/task/plant gates、fresh replay、non-overwrite、历史回归和fresh-namespace revision reuse全部PASS。
- **DG21-08 / OPEN / REVIEW — claims：** REVIEW确认blocking findings为零，且无NMPC/真机/terrain/real-time越界结论后，Verdict才可为`PASS`。

## Interfaces and Compatibility

- 输入：canonical `RobotState`；versioned nominal model/solver/task/reference/formal profiles；Phase20 equilibrium/reset anchor；10 ms control tick和2 ms world-frame disturbance schedule。
- 内部：`WbcReference`提供base/leg/wheel task target与left/right 12D interaction wrench；`WbcModel`从canonical state重构reduced dynamics；`WbcProblem`固定矩阵/order/scale；`WbcSolution`提供candidate、residual、margin、task/slack/solve diagnostics。最终类型名可语义等价，但层次和sign/order不可合并隐藏。
- 输出：canonical `TorqueCommand`六路N·m；additive `StepResult`或等价WBC diagnostics；runner control/plant CSV、summary、manifest、hash和evidence。
- 必须保持：FLU/quaternion/world twist、joint order/sign/unit、`RobotState/TorqueCommand`/ROS schema、Adapter watchdog/fail-zero、Phase16 timing、旧Controller modes、historical evidence和non-overwrite。
- 允许改变：`wheel_leg_core`中增加独立model/QP/WBC模块、config/reference/result字段和tests；`wheel_leg_mujoco`增加独立WBC runner/target；新增Phase21 config、实验工具和文档。任何第三方solver依赖只能在DG21-03关闭后按冻结版本进入。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| P21-T01 | 固化grounding、reuse与差异清单 | Phase14/15/18/20、Simulink WBC、live Core/Adapter | source/graph/coverage记录、可复用与不可复制清单、model/solver/evidence边界 | 候选路径coverage和源码fallback完整；不以旧图或Simulink结果代替当前事实 | done |
| P21-T02 | 冻结12-DoF state与被动重构 | P21-T01、Phase15 branch/reduction、RobotState | reduced-coordinate/spec、passive q/dq reconstruction、workspace/conditioning/fault contract | DG21-01第一部分；closure/branch/velocity/finite-difference/symmetry PASS | done |
| P21-T03 | 建立independent nominal dynamics/contact model oracle | P21-T02、Phase14 model、Phase18 contact sites | versioned parameter profile、`M_r/h_r/S_r/J_c/Jdot_nu`与wrench-map oracle、manifest | DG21-01/02；mass symmetry/PD、bias、virtual work、forward/inverse、force/sign/order PASS | done |
| P21-T04 | 冻结QP数学与solver路径 | P21-T03、historical36D contract、candidate solvers | exact variable/matrix/scale spec、solver audit/benchmark、golden QP与failure corpus | DG21-03；42D candidate从零冻结，旧36D结果仅历史 | done |
| P21-T05 | 逐层验证hard constraints与equilibrium | P21-T03/T04、Phase20 equilibrium | equality-only、torque、H-cone、acceleration及state-dependent joint-protection decision分层结果与margins | DG21-04；旧36D结果仅历史 | done |
| P21-T06 | 冻结weighted standing tasks与wrench/slack | P21-T05、Phase20 case envelope、tuning/holdout split | task definitions/scales/weights、equilibrium wrench、slack contract、ablation/attribution与formal candidate profile | DG21-05；逐task启停方向、竞争、slack、10 s nonlinear tuning/holdout PASS | done |
| P21-T07 | 实现runtime-independent C++ model与solver组件 | P21-T02～T06冻结spec | model/reconstruction、fixed-order QP assembly、solver wrapper、golden/property/fault tests | 与offline oracle逐项一致；production library无MuJoCo依赖；hard reject语义PASS | done |
| P21-T08 | 实现additive WBC Core mode | P21-T07、canonical Core | opt-in mode、internal reference、solution diagnostics、zero/latch/reset/limit逻辑 | DG21-06；state→problem→solution→torque、timestamp、fault和旧mode回归PASS | done |
| P21-T09 | 实现full-3D WBC loop与日志 | P21-T08、Adapter、Phase20 loop | 独立C++ runner/CMake target、control/plant CSV、disturbance/fault/reset/ZOH入口 | runner只经Core↔Adapter；5-step ZOH、双时钟、plant-truth隔离、replay PASS | doing |
| P21-T10 | 建立正式方法、profiles与evaluator | P21-T06/T09 | `docs/experiments/mujoco_weighted_wbc_validation.md`、versioned configs/wrapper/evaluator/manifest schema | formal前freeze、non-empty拒绝、hash/schema/case/threshold/solver字段完整 | todo |
| P21-T11 | 执行formal、fresh replay与历史回归 | P21-T10 frozen inputs | 新`evidence/automated/<run-id>/`、summary/manifest/replay/reuse audit | DG21-07；normal/perturbation/fault、QP/task/plant、replay、non-overwrite与Phase14/15/18/20回归PASS | todo |
| P21-T12 | REVIEW | 全部任务、源码与真实evidence | `REVIEW.md`；仅PASS后创建`RECORD.md` | DG21-08关闭、blocking findings=0后才更新ROADMAP complete | todo |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Pre-freeze Layers

1. **State/reconstruction：** equilibrium和Phase15 workspace samples上的passive q/dq、closure、branch continuity、condition number、orientation tangent和canonical signs。
2. **Model：** `M_r` symmetry/positive-definiteness、`h_r` gravity/Coriolis、`S_r` actuator work、contact Jacobian/bias、forward/inverse residual、finite difference、virtual work和左右mirror。
3. **QP/solver：** zero/equilibrium/random-feasible/active-torque/active-friction/infeasible/ill-conditioned/non-finite corpus；比较独立offline oracle，检查primal/dual/stationarity/complementarity、iteration、warm-start和重复运行exact/tolerance规则。
4. **Hard boundaries：** 按冻结row order逐项累积dynamics equality、torque box、左右37-row H-cone和12D acceleration box；每层保存feasibility、margin与active-row归因。current nominal没有authoritative joint limits，故不臆造state-dependent protection，禁止用最终轨迹掩盖中间错误。
5. **Soft tasks：** 依次加入contact acceleration、base height/orientation、rolling/X anchor、leg posture、wrench/slack与regularization；做single-task、ablation、weight/scale sweep和gradient/task attribution。
6. **Nonlinear pre-freeze：** tuning与未参与选参的正负/组合holdout分离；全部使用full nonlinear plant和至少10 s window，通过后才修改Core。

### Automated

- Phase执行时新增的Python model/QP/prefreeze/formal工具先运行`python -m py_compile`，所有命令与真实结果写回对应任务和evidence。
- `./.venv/bin/python tools/experiments/<phase21-model-validator>.py --output-dir data/experiments/<new-phase21-model-id>`：DG21-01/02 reduced model、contact/wrench oracle。
- `./.venv/bin/python tools/experiments/validate_weighted_wbc_static_attribution.py --config simulation/mujoco/config/phase21_static_attribution.json --output-dir data/experiments/<new-phase21-static-attribution-id>`：复现static corpus、workspace scope及torque/contact/dynamics bounded attribution；仅为反事实诊断，不改变冻结物理contract。
- `./.venv/bin/python tools/experiments/validate_weighted_wbc_base_contact_closing.py --config simulation/mujoco/config/phase21_base_contact_closing.json --output-dir data/experiments/<new-phase21-base-contact-closing-id>`：固定7个blocker与matched controls，执行required-wrench/facet/nearest-cone/base-row/full-plant static/dynamic closing attribution；不组装QP。
- `./.venv/bin/python tools/experiments/<phase21-qp-validator>.py --output-dir data/experiments/<new-phase21-qp-id>`：DG21-03/04 solver和layered hard constraints。
- `./.venv/bin/python tools/experiments/<phase21-prefreeze>.py --output-dir data/experiments/<new-phase21-prefreeze-id>`：DG21-05 weighted task、slack和nonlinear tuning/holdout；通过前禁止Core集成。
- `cd ros_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-up-to wheel_leg_mujoco && colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco && colcon test-result --verbose`：production build、model/QP/Core/Adapter/ROS/runner和旧mode兼容测试零失败。
- `./.venv/bin/python tools/experiments/<phase21-formal>.py --output-dir data/experiments/<new-phase21-formal-id>`：冻结formal matrix；fresh replay按manifest exact/tolerance规则通过。
- `./.venv/bin/python tools/maintenance/test_mujoco_coordinate_contract.py`及Phase14/15/18/20各自reuse入口：canonical、model、kinematics、contact、simple-standing历史回归，不覆盖旧evidence。
- `git diff --check`：Phase实现和文档无空白错误。

命令中的尖括号是执行阶段由P21-T03/T04/T10冻结的真实文件名/run-id占位符；PLAN阶段不创建伪脚本或伪结果。

### Formal Matrix / Evidence

- 至少复用Phase20的nominal reset/replay、正负pitch/velocity/yaw-rate、world X/Y force、roll/pitch/yaw moment、正负组合holdout和6个fault cases；normal/perturbation每case至少10 s。
- 新增QP层case：equilibrium、单task/双task竞争、active torque、near-friction、wrench step、large-slack但feasible、hard infeasible、solver timeout/iteration/non-finite、cold/warm start和reset replay。
- 每个control tick记录model/reconstruction status、solver status/iteration/time、`nudot/tau/lambda/slack`、hard equality/inequality/KKT residual、active set、friction/torque margin、normalized task residual/cost和fail-closed原因。
- plant侧独立记录state、contact、normal load、penetration、rolling/lateral slip、closure和external wrench；明确这些字段只用于验证，未进入Controller输入。
- 正常case必须finite、bilateral contact、无hard violation/solver fault/torque saturation/deadline miss，base/leg/rolling tasks在冻结门槛内恢复；fault case必须当tick或冻结延迟内six-torque zero并锁存到reset。
- `QP feasible`、small slack、task cost、plant stability分别报告；REVIEW不得用其中一项替代其他gate。plot/animation只用于观察，不是PASS证据。

## Acceptance Criteria

- [ ] 12-DoF reduced coordinate、passive reconstruction、world-tangent dynamics和contact/wrench signs经独立oracle验证，runtime无MuJoCo/private-state依赖。
- [ ] 当前contact representation对应的problem具有明确dimension/scale/order/residual/margin，solver candidate通过独立golden/fault/deadline验证；历史36D结果不得继承。
- [ ] soft contact、standing motion、wrench/slack和regularization按归一化weighted contract逐层通过；weights由tuning/holdout证据冻结，不复制Simulink或Phase20数值充当authority。
- [ ] additive WBC mode、内部future-NMPC reference边界、diagnostics、fail-zero/latch/reset、2 ms/10 ms/5-step ZOH和旧mode兼容全部通过。
- [ ] formal至少覆盖Phase20同等级19个10 s normal/perturbation和6个fault cases，plant safety gate不弱；QP/KKT/task/slack/deadline新增gate全部PASS。
- [ ] fresh replay、non-overwrite、Phase14/15/18/20历史回归和fresh-namespace revision reuse通过；manifest记录model/profile/solver/source/binary/config/seed/case/threshold/input/output hash。
- [ ] 失败、fallback、large slack、active constraints和saturation的解释彼此分离；结论明确仅限current nominal simulation，不声明真机、NMPC、terrain或target hardware real-time。
- [ ] REVIEW=`PASS`且blocking findings为零后才创建RECORD并把ROADMAP改为complete。

## Execution Notes

- 2026-08-26：Phase 21 PLAN建立；当前仅制定范围、数学/接口边界、任务和放行门，不执行实现或仿真。
- 2026-08-26：CBM和live source确认production C++尚无QP、linear algebra、reduced dynamics或WBC实现；Simulink 36D WM-WBC只作为semantic/algorithm baseline，不作为current production事实。
- 2026-08-26：冻结standalone standing-first路线和future-NMPC内部wrench/slack接口；solver实现/依赖、model数值profile、task scales/weights与formal阈值必须由DG21-01～05真实证据关闭。
- 2026-08-26：用户要求执行Phase 21；ROADMAP推进为`active`，P21-T01开始。production Core仍受DG21-01～05 pre-freeze gate约束。
- 2026-08-26：P21-T01完成；grounding记录见`evidence/grounding.md`。P21-T02开始，先冻结12D tangent与被动重构，不触碰Core。
- 2026-08-26：model-oracle-v1真实执行为FAIL：除contact velocity finite-difference外其余当前gates通过；发现world-vertical offset不是material point，却错误比较material-point Jacobian。保留v1结果，新增superseding v2，恢复Phase15轮体局部`[0.05,0,0] m`接触点语义，不放宽阈值。
- 2026-08-26：model-oracle-v2全部当前gate PASS并关闭P21-T02；冻结12D world-axis tangent、canonical active sign/order、passive fail-closed contract和Phase15 material contact point。P21-T03继续补齐velocity-dependent bias、`Jdot_nu`和wrench map。
- 2026-08-26：model-oracle-v3为FAIL：新增Coriolis power与wrench map通过，但`Jdot_nu`二阶位置oracle不收敛；原因是验证器用初始`N(q)`单步积分，未沿`qdot=N(q)nu`更新reduction。保留v3结果，v4改用分段flow积分且不改变gate。
- 2026-08-26：model-oracle-v4仍为FAIL，但二阶误差按Euler子步数从约`1e-3`降至`5.03e-5 m/s^2`，确认是验证积分截断误差；v5改用显式中点flow积分，不放宽`2e-5 m/s^2` gate。
- 2026-08-26：model-oracle-v5全部gate PASS，P21-T03完成；`Jdot_nu`二阶误差最大`5.84e-8 m/s^2`，Coriolis power identity最大`1.21e-14 W`，wrench map最大误差`2.78e-17`。P21-T04开始并冻结Eigen-only fixed dense ADMM候选，Core集成仍等待solver corpus。
- 2026-08-26：组装equilibrium hard dynamics时发现v5局部gate缺少静态可实现性：单轮3D material-point force模型的`[S,J_c^T]` rank为11，最小静态residual约`0.626`且集中在base pitch moment。撤回P21-T03完成状态，新增Phase18 contact resultant/COP oracle；该冲突不得由task slack、调权或torque clip掩盖。solver组件审计可独立继续，但Core集成保持blocked。
- 2026-08-26：Phase20 equilibrium每轮实际有3个mesh contact；其resultant可由单一COP force表示（force-parallel moment约`5e-14 N·m`）。v6使用各侧force-weighted COP转到wheel-body local frame，并新增zero-acceleration static dynamics gate；这属于current nominal profile，不能外推到新CAD/contact profile。
- 2026-08-26：model-oracle-v6 static gate FAIL=`2.34e-4`，定位为COP profile仅保留8位小数；v7保存完整double precision COP，继续使用原`1e-7` residual gate。
- 2026-08-26：v7证明残差并非小数截断，而是Phase20 compliant closure与Phase21 ideal reduced closure的compatibility差；v8对两侧effective COP local-X共同校正`-18.68650895129817 um`，该值只属于current nominal profile且必须接受nonlinear holdout/revision reuse检验。
- 2026-08-26：model-oracle-v8全部gate PASS，equilibrium static dynamics residual=`1.89e-14`，P21-T03重新关闭；失败v1/v3/v4/v6/v7全部保留。P21-T04 solver corpus继续，P21-T05尚未开始。
- 2026-08-26：QP prefreeze-v1为FAIL：equality/torque/contact层收敛且cross-oracle差`1.66e-8`，但加入inactive acceleration bounds后20k iteration未满足过严dual tolerance，Python solve time也超过10 ms。保留v1，v2调整ADMM `rho/tolerance`并继续以hard residual和C++ reference-host benchmark为authority。
- 2026-08-26：QP prefreeze-v2 (`rho=0.1`)恶化primal收敛并FAIL；v3测试`rho=10`。所有失败profile保留，不使用adaptive rho掩盖结果。
- 2026-08-26：v3仍FAIL；确认variable scaling后box/friction rows未归一化，inactive constraints被重复放大。v4把各物理bound row归一到O(1)，保持原可行域、物理limits与hard residual gate不变。
- 2026-08-26：QP prefreeze-v5把C++ benchmark并入manifest后全部gate PASS；四层hard QP均收敛，最大hard residual=`5.80e-8`、最大stationarity residual=`5.52e-8`、cross-oracle差=`1.68e-7`，infeasible corpus未被接受。固定求解器1000次cold p99=`1.80011 ms`、cold max=`2.888224 ms`、warm=`0.00344 ms`。P21-T04/T05及DG21-03/04关闭，P21-T06开始。
- 2026-08-26：weighted-task v1冷启动在tick 116触发iteration limit并fail-zero；v2用冻结的warm-start/rho路径消除早期数值失败，但强wrench fidelity导致2 s X漂移`4.60 cm`。v3/v4证明单纯提高base-X weight只会放大任务冲突。
- 2026-08-26：task v5把wrench-slack penalty从`100`降至`1`，2 s除settling外全部gate通过，但完整10 s nominal在tick 272进入maximum-iteration/infeasible并失稳。v6/v7/v9的base-X weight/gain/damping sweep均未消除失败。
- 2026-08-26：model v9把固定wheel-material COP改为每状态重选的equilibrium-calibrated world-offset selector并通过其local oracle；task v8仍未通过nonlinear nominal。由此撤回DG21-01/02的contact部分与P21-T03完成状态，P21-T06标记blocked；证据见`evidence/task_prefreeze.md`。禁止继续P21-T07 model/Core integration或用slack/调权掩盖contact model错误。
- 2026-08-27：完成不调参的12-case failure-attribution audit。修复仅用于审计的task enable接线/empty-set缺陷，逐tick记录task direction/cost、active bounds和MuJoCo mesh-contact resultant/COP/reduced generalized-force mismatch，并保存tick 240～272完整QP。baseline tick 272的同一QP经HiGHS 33/33判为可行、SLSQP 33/33求解成功；禁用wrench fidelity只延后2 tick，五个single-task中位方向均为正，而contact mismatch随COP/rolling/task residual在失败前增长。结论为contact/model mismatch是上游blocker、ADMM iteration limit是下游直接故障；DG21-01/02仍REWORK、P21-T03仍doing、P21-T06仍blocked，证据见`evidence/failure_attribution.md`。
- 2026-08-27：执行rolling-contact representation decision audit。固定点/几何centroid的moment residual分别最大`0.871/0.617 N·m`；lowest-four仅67.78%可表示，four-extreme为99.44%，1 mm full support patch与deterministic lowest-eight均在537个pre-failure wheel-side样本上以`1e-7`阈值100%表示truth resultant。冻结lowest-eight、每点3D force patch仅作为下一local model-oracle候选；它将QP从36改为78变量，因此不得继承DG21-03/04的旧solver/hard-QP PASS，且contact Pfaffian/Jdot仍开放。证据见`evidence/rolling_contact_representation.md`。
- 2026-08-27：执行frozen lowest-eight local-oracle第一门。fresh canonical-state capture与独立replay仍给出540/540 wrench representability，但rolling selector发生left/right `9/10`次真实集合换点；同姿态槽位位置/Jacobian最大跳变为`44.93 mm/0.0403`，10 ms bias jump proxy为`14.53 m/s²`，因此无法定义连续`Jdot_nu`。按gate立即REWORK，未运行Pfaffian/static、78D hard-QP或benchmark；DG21-03/04重开为REWORK，P21-T04/T05/T06 blocked。证据见`evidence/lowest_eight_patch_local_oracle.md`。
- 2026-08-27：冻结解析continuous contact frame与最小geometry-complete六点surface patch。四个band corners在original corpus仅`534/537`而被拒绝；增加两bottom lateral endpoints后original/fresh为`537/537`与`540/540`。geometry velocity/bias误差`1.23e-7/5.01e-7`，13,944次force-map最大`1.11e-15`，`G:18→6`在566状态始终rank6/nullspace12。独立每轮三行soft Pfaffian以实际capture速度验证，velocity/`Adot_nu`误差`1.52e-7/2.84e-6`。representation local gate PASS，但projected-wrench/static dynamics仍开放，未进入QP/solver/Core；证据见`evidence/continuous_contact_representation.md`。
- 2026-08-27：contact-centered condensation oracle证明六点相对坐标及`G_C`最大变化`1.28e-16`，固定24-ray/37-row cone在1,240个point/V/H cases中完全一致，original/fresh truth `1077/1077`、generalized-force/virtual-work均PASS，DG21-02关闭。但static gate三条独立表示均仅`122/173`可行，51个失败覆盖random_04和selected reconstructed rolling ticks 212～271；保留point/internal-force变量也不能修复。DG21-01/P21-T03继续REWORK/doing，42D hard-QP candidate不授权，证据见`evidence/contact_centered_wrench_condensation.md`。
- 2026-08-27：完成static failure bounded attribution并复现raw `122/173`。按已冻结Phase15 componentwise workspace复核后，129个envelope内状态为`122 feasible / 7 failed`，其余44个是envelope外fail-closed coverage（最大component ratio `15.6073`）。7个nominal失败在condensed-H与point-force两种unbounded torque-scale问题中都不可行，移除torque bounds无效，而移除contact cone全部修复；反事实minimum-`mu`中位`15.3466`、最大`19.4758`，不授权修改冻结`mu=1`。blocker收敛为base-equilibrium/contact compatibility；Phase仍REWORK，42D/QP/Core继续停止，证据见`evidence/static_failure_attribution.md`。
- 2026-08-27：base/contact closing oracle保持全部frozen contact/model参数，对7个blocker及4个matched controls保存required wrench、37-row facet、nearest-cone、base/active/full rows和same-q full-plant rigid static LP。7个blocker均先在base rows失败且只违反H-row28的positive pitch-moment support bound；plant同姿态也全部static-infeasible，`random_04`仅右轮接触。Phase15/20 contract复核证明workspace不承诺static support；rolling dynamic probe `8/8`通过，最坏absolute/relative residual=`1.39348/0.0221514`。7个状态均分类A，DG21-01关闭、P21-T03完成；42D hard-QP仅获准作为P21-T04下一candidate，未进入QP/solver/Core。证据见`evidence/base_contact_compatibility_closing.md`。
- 2026-08-27：从零冻结42D hard-QP：`[nudot12,tau6,wL_C6,wR_C6,slackL_FLU6,slackR_FLU6]`，12行dynamics、6行torque、左右各37行H-cone及12行acceleration，共104行；future fidelity slack不进入hard rows，state-dependent joint protection确认尚未冻结并保留为P21-T05 gate。4个workspace与28个rolling dynamic问题全部通过HiGHS/SLSQP独立oracle，最大violation/stationarity=`1.12e-15/2.19e-15`，最坏scaled normal-matrix condition=`17.438`。42D Eigen-only dense ADMM的cold/repeated-warm/dynamic-warm各1000次全部PASS，最大oracle差`3.70871e-7`、hard residual`8.71323e-8`、cold/dynamic total setup+solve最大`1.455389/1.556428 ms`，20项tests零失败。DG21-03关闭、P21-T04完成；证据见`evidence/hard_qp_42d.md`与`evidence/solver_audit_42d.md`。
- 2026-08-27：P21-T05对同一32-case corpus执行42D累计四层oracle，`32/32`在dynamics、torque、H-cone和acceleration各层全部可行；128个QP最大bound/stationarity/complementarity=`5.47e-14/1.32e-13/2.62e-17`，physical dynamics最大残差=`6.50e-13`，hard violation/slack均为零。Phase20 equilibrium追加12行zero-`nudot`后116-row static problem通过，minimum cone/torque margin=`0.310102/1.99854 N·m`。六个active joints均`limited=false, range=[0,0]`，本profile明确排除未获authority的state-dependent joint protection并保留12D acceleration box。DG21-04关闭、P21-T05完成，P21-T06解锁；证据见`evidence/hard_feasibility_42d.md`。
- 2026-08-27：在DG21-01～04关闭且P21-T03～T05完成后，P21-T06进入`doing`。本轮只允许从冻结42D hard contract重新建立weighted task、normalization、equilibrium wrench/slack、ablation和tuning/holdout nonlinear evidence；旧36D/single-force task权重、wrench penalty及其nonlinear结果仅作反例和语义回归，不得直接继承。DG21-05关闭前继续禁止P21-T07/Core集成。
- 2026-08-27：新的42D weighted-task local oracle在不改104-row hard contract下全部PASS：32/32 corpus最大hard violation=`4.44e-16`，equilibrium最大`|nudot|/|slack|/wrench normalized residual=2.65e-4/4.74e-5/8.29e-7`，54个双向task-component probes最小正向响应=`0.0308124`，wrench从contact center平移到base-control-site并转body-aligned FLU的cross-check与slack符号误差均为零。该结果只接受task algebraic/local candidate；equal weights、KP/KD和wrench/slack tradeoff尚未冻结，P21-T06保持doing、DG21-05保持OPEN，证据见`evidence/task_prefreeze_42d_local.md`。
- 2026-08-27：在未运行42D holdout前完成task competition/accounting与4个10 s tuning cases：32-case baseline/逐task-disable全部可行且每项own-task attribution为正，H/g重建误差为零，objective accounting误差`1.07e-14`，实际slack block为`-I_12`且不进hard rows。冻结equal weights、PD gains、tick-100扰动时序和normalized slack/task residual/task cost envelopes=`0.01/0.02/0.001`；4个tuning cases全PASS，solver/nonfinite/saturation均为零。详见`evidence/task_prefreeze_42d_nonlinear.md`；P21-T06/DG21-05仍等待冻结输入下的9个holdout。
- 2026-08-28：用未改变的frozen profile/runner执行并逐manifest审计9个10 s holdout，`9/9`全部PASS；最大hard/bound violation=`2.22e-16`，solver/nonfinite/saturation均为零，最坏normalized slack/task residual/task cost=`0.003535/0.005452/0.0001502`，均低于holdout前冻结门限。中途子代理额度中断仅造成串行命令续跑，不是模型或evidence FAIL。P21-T06完成、DG21-05关闭并解锁P21-T07，详见`evidence/task_prefreeze_42d_nonlinear.md`。
- 2026-08-28：P21-T07 live grounding确认production tree已有42变量/128-row Eigen-only `DenseQpSolver`且基线build与2项tests通过，但尚无runtime-independent model/reconstruction/QP assembly。Codex冻结两步实现路线：先由Claude生成并复核compiled numeric runtime profile，再实现analytic 16D-tree→12D reduction、42D problem和solver wrapper；不触碰ControllerCore。Claude CLI因用户级外部token包含非ASCII而在模型调用前失败，排除用户settings后又无登录态；按临时worker policy将P21-T07标记blocked，环境修复后从Step A继续。交接见`evidence/p21_t07_implementation_handoff.md`。
- 2026-08-28：用户授权本轮不使用Claude、由Codex继续P21-T07。Step A deterministic runtime profile两次fresh生成字节一致，11-body inertial cross-check为零；analytic C++ model在四个workspace与dynamic tick 68/204对fresh golden逐项一致。但冻结`dynamic_tick_271`的hip相对equilibrium偏移`1.2168/1.2486 rad`，超出Phase15 `[-0.65,0.65]` workspace；runtime按冻结contract必须fail closed，而P21-T04/T05及handoff又要求其作为nominal 32-case parity输入。该authority冲突使P21-T07继续blocked，禁止绕过workspace gate或宣称32-case parity；详见`evidence/runtime_workspace_gate_audit.md`。
- 2026-08-28：修复保持Phase15 workspace不变，将完整capture先按workspace分类；tick 1–259组成eligible set，按预声明等距规则冻结28个dynamic nominal cases，tick 260/271转为rejection corpus。hard、tasks、competition及13个nonlinear tuning/holdout全部fresh PASS且workspace violation为零。C++ analytic model、42D/104-row assembly、`alpha=1.6/rho=0.15` solver wrapper完成32-case golden与1000-run deadline审计；5个wheel_leg_core tests全过，P21-T07 done，详见`runtime_workspace_gate_repair.md`与`runtime_cpp_parity.md`。
- 2026-08-28：P21-T08完成additive `kWeightedWbc` Core mode。冻结producer在equilibrium与P21-T07 golden reference逐项一致，Core torque与独立cold controller在`1e-12 N.m`内一致；10 ms warm/reset、contact/timing/六维workspace、invalid/non-monotonic、torque-limit fail-zero/latch及旧`kZero`回归全部通过。Release build和6/6 CTest PASS；该结果只关闭Core部分，DG21-06仍等待P21-T09的Adapter、5-step ZOH、双时钟和runner证据。详见`evidence/runtime_core_integration.md`。

## Blockers

**Current blocker：** none。P21-T08 Core contract已通过；P21-T09正在建立独立full-3D runner、日志、5-step ZOH、fault/reset与replay入口，formal结论仍未授权。
