# Phase 19 v1: nominal 平面简单站立（固定腿姿态 + 轮式平衡）— PLAN

Status: `review`

## Goal

在不连接真机、不引入 WBC/QP/NMPC 的前提下，使用 current nominal MuJoCo 完整多刚体 plant、Phase 18 wheel-only contact 和 2 ms/10 ms/5-step 确定性链路，实现固定对称腿姿态下的 sagittal 简单站立：双轮持续接地、机身 pitch 收敛、base height 与腿姿态有界、common wheel rolling position 不持续漂移，并在小扰动、限幅、reset/replay 和非覆盖复现矩阵中通过 simulation-only 审查。

## Current State

- 已有：Phase 02/04 冻结 canonical FLU、六关节顺序、`q_C=-q_M+b`、`dq_C=-dq_M`、`tau_M=-tau_C`、base COM-site pose/twist、命名 wheel-floor contact 和 Adapter watchdog/reset；`RobotState` 已包含 base position/quaternion、world linear/angular velocity、六关节 q/dq 和左右二值 contact。
- 已有：Phase 15 冻结 current nominal 闭链几何、被动分支、reduced Jacobian、轮轴 `+Y`、轮半径 `0.05 m`，以及正 canonical wheel rotation 对应无滑 `+X` rolling 的方向契约。
- 已有：Phase 16 提供唯一 C++ Controller↔Adapter↔MuJoCo deterministic loop、`0.002 s` physics、`0.010 s` control、5-step ZOH、双时钟、fail-safe、reset/replay 与 append-only 日志基础。
- 已有：Phase 17 的 Controller Core 支持 default-zero 与显式 Joint PD+current nominal reduced gravity、静态 reference、逐关节求和后限幅及 diagnostics；其正式证据仅覆盖 fixed-base/contact-disabled，wheel PD 不能直接用于允许持续滚动的平衡轮。
- 已有：Phase 18 建立 `phase18_floating_contact.xml`、wheel-only collision/contact profile 和完整模型 zero-command free-flight/touchdown；`0.2 s` authority 证明接触 plant/state/reset 可用，但长时间零控制会倒塌，不是站立证据。
- 已有对照：Simulink baseline 含 floating-base LQR、wheel-position reference、简化刚体与后续 QP/WBC。它可以提供状态、符号和验证思路，但其简化质量/惯量、wrench 输入和 contact 模型不同，不能直接复制 gain 或把 Simulink PASS 当作 MuJoCo PASS。
- 缺少：contact-aware 静态站立平衡点、floating-base 支撑前馈、可审计的 pitch/common-wheel 状态、平面局部控制模型、站立 Controller mode、floating/contact C++ 闭环入口，以及小扰动下的正式稳定性证据。
- Grounding：live code 以 CBM project `W_L_ws` 当前 generation 和直接源码为准；`adapter.hpp:28` 的 partial coverage 已通过直接读取补齐。Graphify 仅查询现有本地图，没有执行 extract/update。

## Scope

- Ground Phase 17 Core、Phase 16 C++ loop、Adapter floating/contact state、Phase 18 scene/profile 和 Simulink 平衡相关函数，形成职责、数据流和最小改动边界。
- 建立新的 `phase19_standing.xml` 和 `phase19_nominal.json`，显式继承 Phase 18 wheel-only contact、solver 和 timestep，保存 current nominal model revision、站立初值、静态 reference、控制参数、case matrix、thresholds 与 hashes。
- 在完整 MuJoCo 多刚体/contact 模型中离线求解并验证一个对称、双轮接触、零速度的 nominal standing equilibrium；记录 base pose、主动/被动关节、每轮法向载荷、closure、所需四路 leg torque 和求解残差。
- 把 Phase 17 leg controller 扩展为 floating standing 的固定姿态控制：hip/knee 使用 `tau_g(q)+tau_support_eq+PD`；wheel 的 Phase 17 position PD/gravity 输出在 standing mode 中禁用，由独立 common-mode balance command 接管。
- 冻结平面状态：`x_s=[x-x_ref, dx, theta-theta_ref, dtheta]`。`x/dx` 直接使用 canonical `RobotState.base_position_n_m[0]` 与 `base_linear_velocity_n_m_s[0]`，reset 时锚定 `x_ref`；`theta/dtheta` 从 canonical base quaternion 与 world `+Y` angular velocity得到，并与 MuJoCo site、轮心和 no-slip oracle 交叉验证。
- 在 nominal equilibrium 附近建立 10 ms 离散局部模型和 common-wheel feedback。模型 authority 为完整 MuJoCo plant 在 leg posture loop 闭合后的数值局部模型；Controller runtime 只加载 versioned gain/coefficient，不链接或调用 MuJoCo。
- 在 Controller Core 增加显式 opt-in `simple_standing` mode、standing config/reset anchor、leg/balance torque 分量、saturation、armed/trip diagnostics；default-zero、Phase 17 mode 和公共 `RobotState/TorqueCommand` schema 保持不变。
- 兼容扩展 Phase 16 C++ runner，支持 floating/contact scene、standing mode、初始 base pose/twist、2 ms 外部 base wrench/impulse schedule 和可选 physics-step contact日志；Python wrapper 只编排、评价、汇总和 manifest，不重写第二套 Controller↔Adapter physics loop。
- 分离 exploratory tuning 与 frozen formal holdout；正式验证 equilibrium hold、正负 pitch/rolling 初始偏差、正负 longitudinal/pitch disturbance、腿姿态扰动、限幅/contact-loss fail-closed、左右共模对称、reset/replay 和跨进程确定性。
- 回归 Phase 02/04/14/15/16/17/18，固化 controller/model/contact/config/runner/output hashes、失败样本、reuse contract 和跨 SolidWorks/identified profile 的非覆盖入口。

## Out of Scope

- 真机上电、STM32/树莓派联调、IMU/encoder 实测、吊架/E-stop、执行器/摩擦/接触辨识或任何 MuJoCo–real 一致性结论。
- Roll、yaw、turning、左右差分 wheel torque、单轮支撑、斜坡、台阶、不平地、跳跃、行走、速度/轨迹跟踪或大范围抗跌倒。
- 自动起立、自由落体后自行站起、跌倒恢复、接触切换规划或超出冻结小扰动 envelope 的 region-of-attraction 声明。
- 显式 Cartesian `z` force controller、interaction-wrench 分配、inverse dynamics、contact-force optimization、QP、Weighted WBC 或 NMPC；本 Phase 的 `z` 只由固定腿姿态间接维持并作为 plant outcome 检查。
- 在线完整多刚体动力学、在线 MuJoCo linearization、runtime `qfrc_bias`、Jacobian/QP 求解或把 MuJoCo object 泄漏进 Controller Core。
- Integral action、anti-windup、gain scheduling、状态观测器/Kalman filter、command filter、rate limiter 或动态 reference/planner；若纯状态反馈存在不可接受稳态偏差，必须 REWORK 或新 Phase，不能顺带增加控制层。
- 新 ROS topic/message/service、standing launch、在线动态参数或 reference source arbitration；Phase 19 只通过 deterministic runner 验证 Core，现有 ROS default-zero/Phase 17 静态行为只做回归。
- 修改 canonical frame/sign/order、公共 message、Phase 18 collision/contact profile、Phase 16 timing、历史正式 evidence 或把 nominal simulation torque limit 写成真机安全限制。
- 直接复用 Simulink 的 LQR gain、简化刚体参数、接触参数或 QP/WBC 输出；Simulink 仅作结构与趋势对照。

## Frozen Decisions

- authoritative plant 是 current nominal 完整 MuJoCo 多刚体模型和 Phase 18 wheel-only contact profile。控制器可以使用降阶局部模型，但所有最终站立结论必须来自完整 contact plant 的闭环运行，不得由降阶模型仿真代替。
- 本 Phase 只做 sagittal/common-mode：左右 hip/knee reference、gains 和支撑前馈按同类对称；左右 wheel torque 必须相等。定义 `tau_common=tau_left_wheel=tau_right_wheel`，差分项严格为零。roll/yaw/Y 漂移只做泄漏监测，不宣称被控制。
- physics/control timing 固定为 `0.002 s / 0.010 s / 5-step ZOH`。Controller 只在 10 ms state sample 后更新，六路 torque 在之后 5 个 physics step 严格 ZOH；接触、penetration 和外部 impulse 按 2 ms physics row 判断。
- 正式站立从已求解、已接触的 equilibrium reset 开始；不把 Phase 18 的 free-fall touchdown 接到站立控制器，也不要求自动接管或起立。reset 时先复位 MuJoCo/Adapter，再清除 Controller 时间、rolling origin、trip latch 和旧 command。
- equilibrium 固定为 upright `theta=0`、双轮接触、零速度和 `tau_left_wheel=tau_right_wheel=0`；允许在 Phase 15 已验证工作域内选择新的对称 hip/knee reference，使整机 COM/支撑几何满足该条件，但必须记录其与 Phase 17 nominal reference 的差异。
- `theta=0` 表示冻结的 upright base frame；pitch 正方向遵守 canonical world `+Y` 右手定则。正式 envelope 远离 Euler 奇异点，pitch 与 pitch rate 必须分别由 quaternion/site orientation 和 angular-velocity oracle 验证，不能靠动画判符号。
- longitudinal state 直接复用已有 canonical base world-X position/velocity，reset 时令当前 `x` 为 `x_ref`，不在 Core 内复制 Phase 15 forward kinematics。正式运行另行记录左右 wheel q/dq、wheel-center world `X`、`r=0.05 m` rolling residual 与 slip；base-X、轮心和 canonical wheel torque 的方向不一致或持续 gross slip 直接失败。
- leg command 固定为 `tau_leg=tau_g_phase17(q)+tau_support_eq+Kp(q_ref-q)+Kd(0-dq)`；对每个 hip/knee，`tau_support_eq=tau_leg_eq-tau_g_phase17(q_eq)`，使 equilibrium 处 PD 为零且命令精确回到 contact-aware 静态解。wheel 不使用 Phase 17 position PD；wheel gravity微项可记录但不得与 balance command重复计入。Phase 17 gravity 对非零 base pitch 的近似误差必须在冻结 standing envelope 内由完整 plant 扰动结果约束。
- balance command 为无积分的 10 ms 离散状态反馈：`tau_common=-K*x_s`，`x_s=[x-x_ref,dx,theta,dtheta]` 按上述顺序。`K`、离散 `A/B`、Q/R 或等价设计输入、闭环 poles、controllability、拟合/线性化残差和有效扰动范围必须进入 profile/manifest。
- local model 必须由完整 nominal MuJoCo/contact plant 在已闭合 leg posture loop 和 equilibrium 附近生成，并以未参与拟合的正负小扰动验证。若 4-state projection 的残差/闭环鲁棒性不满足预冻结 gate，本 Phase 进入 REWORK；不得静默扩张为 full-state WBC。
- torque clamp 在 `leg gravity + support + PD` 或 wheel balance command 形成后逐关节对称执行。simulation-only hip/knee/wheel limit、最大速度、最大 pitch、最小 height 和 contact-loss 条件都在 formal 前冻结；触发 envelope/contact/finite fault 时 standing mode fail closed，输出零力矩并锁存到 reset。
- `z` 不进入 balance state，也没有独立 vertical force loop。base height 由固定 leg geometry、leg PD/support feedforward 和 wheel contact 间接维持；height、leg error、normal force symmetry 和 contact continuity是正式硬指标。需要主动调高/调低或精确力分配时进入 Weighted WBC Phase。
- Simulink baseline只用于比较 state/order/sign、平衡趋势和控制层边界；由于其简化刚体、wrench interface 和 compliant contact 与当前 MuJoCo 不同，Phase 19 gain/equilibrium/threshold 只从 current nominal profile 生成。
- exploratory、formal、fresh-process replay 与所有回归使用新目录；formal config/hash 冻结后不得放宽 threshold、删除失败 case 或覆盖旧 evidence。SolidWorks/identified revision 使用新 scene/profile/run，并重新求 equilibrium、local model、gain 和 thresholds。

## Open Questions / Decision Gates

- **DG01 / CLOSED / CODEX_DECISION — 控制复杂度：** 本 Phase 使用固定腿姿态 + common-wheel 4-state feedback；完整多刚体只作为 plant 和离线局部模型 authority，不做在线 full-body dynamics/WBC。
- **DG02 / OPEN / REWORK EVIDENCE — contact-aware equilibrium：** 当前候选虽能保持双轮接触，但 10 ms 仿射漂移包含 `0.08638 rad/s` pitch-rate 增量，尚不是满足冻结定义的零轮扭矩静态平衡点。
- **DG03 / OPEN / PARTIAL — standing state contract：** exploratory runner 已直接使用 `base_control_frame` site pose/Jacobian twist、quaternion pitch、world `omega_y` 和 reset anchor；wheel rolling oracle 与 Core/Adapter 端到端映射尚未执行，因为 DG04 已阻断实现。
- **DG04 / OPEN / REWORK EVIDENCE — local model 与 gain：** 模型可控秩为 4，但候选闭环谱半径为 `1.0320567 > 1`；完整 plant 只表现为带偏差的有界运动，未通过局部稳定与恢复 gate。
- **DG05 / OPEN / REWORK EVIDENCE — posture/support/gain/limit envelope：** deterministic seed-19 搜索保留的最佳有界候选仍违反 pitch/x 恢复和 lateral/roll/yaw 泄漏阈值，不能冻结为 formal profile。
- **DG06 / OPEN / NOT ENTERED — simple-standing formal：** pre-freeze gate 失败，按 PLAN 不进入 formal，不实现 Core standing mode，也不把诊断性平面稳定外力当作站立证据。
- **DG07 / OPEN / PARTIAL — determinism/compatibility/reuse：** exploratory fresh-process timeseries/summary exact，non-overwrite PASS，Phase 19 scene 与 Phase 18 编译维度一致；因未进入实现/正式矩阵，完整历史回归尚未执行。
- **DG08 / CLOSED / SCOPE — z/WBC 边界：** 本 Phase 只间接保持 z；主动 height/wrench/contact-force 分配、roll/yaw 和复杂全身任务明确进入后续 Weighted WBC Phase。
- **DG09 / CLOSED / SCOPE — physical validity：** PASS 仅表示 current nominal simulation 在冻结小扰动 envelope 内站立；不关闭真实执行器、传感器、延迟、轮胎/地面或安全 gate。

## Interfaces and Compatibility

- 输入：canonical `RobotState`；`phase19_standing.xml`；versioned standing profile；静态 leg posture/pitch/rolling reference；initial-state 与 2 ms external wrench/impulse schedule；Phase 18 contact profile。
- 输出：canonical `TorqueCommand`，顺序仍为 `[left_hip,left_knee,left_wheel,right_hip,right_knee,right_wheel]`、单位 N·m、同 source timestamp；standing diagnostics 包含 armed/tripped/reason、`x/dx/theta/dtheta`、leg gravity/support/PD、wheel balance、raw/clamped torque 和 saturation。
- physics-step evidence：time、base/site pose/twist、system COM、左右 wheel-center/contact、contact pair/count/wrench/penetration/slip、closure residual、external wrench 和 finite/solver flags。
- control-tick evidence：保持 Phase 16/17 基础列和语义，只追加 standing state/reference/torque decomposition、contact envelope、trip 与 recovery字段。
- manifest：model/contact/controller/profile/scene/runner hashes、MuJoCo/compiler版本、timing/solver、equilibrium/local-model/LQR 数据、case IDs、seed、thresholds、run ID、`supersedes`、hardware-data=false。
- 必须保持：Core 无 ROS/MuJoCo依赖；公共 `RobotState/TorqueCommand` 不变；Adapter sign/order/watchdog/contact/reset 不变；default-zero 和 Phase 17 mode 行为不变；Phase 16 old scenarios/CSV 列可继续解析。
- 允许改变：ControllerConfig/StepResult 增加 standing mode/config/diagnostics；deterministic runner 增加可选 floating/standing/physics log/base disturbance 参数；新增 Phase 19 scene/config/wrapper/tests/docs/evidence。ROS wrapper 本 Phase 不增加 standing 配置入口。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| T01 | Ground Core↔runner↔Adapter、ROS兼容面、Phase 15–18 contracts 和 Simulink 对照 | CBM/source、Phase RECORD/evidence、现有 Graphify 图 | grounding、职责/数据流/改动影响、DG01/DG08/DG09 关闭记录 | 精确映射状态/力矩/时序/contact；说明 Simulink 可复用与不可复用部分；不运行 Graphify update | done |
| T02 | 建立 Phase 19 scene/profile 与 contact-aware equilibrium，关闭 DG02 | Phase 18 scene/contact、Phase 15 branch、Phase 17 posture | `phase19_standing.xml`、`phase19_nominal.json` equilibrium/provenance | 双轮接触、closure、静态加速度/约束残差、左右 normal force/torque symmetry、有限值和 reset 一致性过阈值 | blocked |
| T03 | 冻结 standing state/sign/reset contract，关闭 DG03 | T01/T02、RobotState、Phase 15 rolling contract | base-X/pitch evaluator、wheel rolling oracle tests、logging schema | quaternion/site/finite difference、wheel-center/no-slip、正负方向、reset anchor 和左右共模全部一致 | blocked |
| T04 | 实现并验证 floating leg posture/support controller | Phase 17 gravity/PD、T02 equilibrium | leg `gravity+support+PD`、wheel-PD禁用、decomposition diagnostics | equilibrium command 与离线解一致；hip/knee正负误差纠正方向、pitch envelope近似、限幅、对称、contact hold 和 default-zero回归 PASS | blocked |
| T05 | 生成 4-state local model 与 common-wheel gain，关闭 DG04 | T02–T04、10 ms sampled full plant | versioned `A/B/K`、design inputs、poles/controllability/fit report | 中心正负扰动、holdout one/multi-step residual、closed-loop poles 和初步 full-plant stabilization 过预冻结门槛 | blocked |
| T06 | 扩展 Core standing mode 与 deterministic floating loop | T03–T05、Phase 16 runner/Adapter | standing config/reset/trip、equal-wheel torque、floating/contact/base disturbance、2 ms physics log | C++ unit/integration tests覆盖公式、sign/order、ZOH、contact loss、invalid/nonmonotonic、reset、saturation；旧 scenario 的既有列语义和数值兼容 | blocked |
| T07 | 建立 Phase 19 wrapper、case matrix、manifest 与 non-overwrite | T02–T06 | `run_mujoco_simple_standing.py`、raw/control/physics CSV、summary/manifest | runner拒绝非空目录；schema/hash/case/threshold完整；Python不复制 physics/control step | blocked |
| T08 | 执行 exploratory envelope 并关闭 DG05 | T02–T07 | gains/limits/trip envelope、扰动幅值、正式 thresholds 与选择记录 | tuning runs 全保留；覆盖正负 pitch/rolling/force/moment/posture；formal config 在 holdout 前冻结 | blocked |
| T09 | 执行正式简单站立矩阵并关闭 DG06 | frozen profile、T07/T08 | equilibrium/perturbation/disturbance/fail-safe raw 与正式 summary | ≥10 s hold、pitch恢复、height/rolling/posture/contact/penetration/closure/torque/finite/泄漏指标全部 PASS；失败样本不删除 | blocked |
| T10 | 执行 replay、历史回归与跨 profile 复用检查，关闭 DG07 | T06–T09、Phase 02/04/14/15/16/17/18入口 | fresh-process comparison、colcon/coordinate/plant/controller regression、revision dry-run | 相同 frozen input 在容差内一致；旧正式 evidence未修改；new model/profile只需新 config/equilibrium/model/gain/run | blocked |
| T11 | 固化方法、reuse contract、evidence 并准备 REVIEW | T01–T10 | 方法文档、README、grounding、automated evidence、Execution Notes、REVIEW输入 | DG01–DG09 全关闭；结论限定 planar/current nominal/simulation-only；没有WBC或真机夸大 | blocked |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Automated

- `cd ros_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-up-to wheel_leg_mujoco`：Core、ROS wrapper、Adapter 和 deterministic runner 在 C++17/Jazzy/MuJoCo 3.7.0 下构建。
- `cd ros_ws && source /opt/ros/jazzy/setup.bash && colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco && colcon test-result --verbose`：standing algebra/state/reset/trip、default-zero、Joint PD、ROS、Adapter/contact 和 runner tests 无失败。
- `./.venv/bin/python tools/experiments/run_mujoco_simple_standing.py --config simulation/mujoco/config/phase19_nominal.json --output-dir data/experiments/<new-phase19-run-id>/raw`：执行 frozen equilibrium/local-model/formal matrix，全部 DG02–DG06 gates PASS。
- 对同一 frozen config 使用两个新的 output directory：规范化 control/physics CSV、summary 与 manifest-derived metrics 在预冻结 determinism tolerance 内一致；再次指向非空目录必须在仿真前失败。
- `./.venv/bin/python tools/maintenance/test_mujoco_coordinate_contract.py`：canonical frame/order/sign、pitch、wheel axis/radius/rolling direction 回归 PASS。
- `./.venv/bin/python tools/experiments/run_mujoco_internal_dynamics.py --output-dir data/experiments/<new-phase14-regression-id>/raw` 与 `run_mujoco_closed_chain_kinematics.py`：Phase 14/15 动力学、闭链、接触点和 reduced Jacobian 回归 PASS。
- `./.venv/bin/python tools/experiments/run_mujoco_controller_loop.py --profile nominal --output-dir data/experiments/<new-phase16-regression-id>/raw`：Phase 16 zero/fault timing/reset/watchdog/replay 回归 PASS。
- `./.venv/bin/python tools/experiments/run_mujoco_joint_pd_gravity.py --config simulation/mujoco/config/phase17_nominal.json --output-dir data/experiments/<new-phase17-regression-id>/raw`：Phase 17 fixed-base Joint PD/gravity 回归 PASS。
- `./.venv/bin/python tools/experiments/run_mujoco_contact_floating_base.py --config simulation/mujoco/config/phase18_nominal.json --output-dir data/experiments/<new-phase18-regression-id>/raw`：Phase 18 wheel contact/free-flight/touchdown 20 gates 继续 PASS。

### Manual / Evidence

- Codex 在实现 standing control 前审查 equilibrium 的完整 qpos/qvel、contact set、normal load、constraint residual 和 generalized force balance；不能用“画面看起来站住”替代静力证据。
- 审查 4-state model 的 state/input convention、数值生成方法、controllability、closed-loop poles、holdout residual 和 sign；Core 中只能出现 versioned ordinary-C++ coefficients，不得出现 MuJoCo runtime oracle。
- exploratory 与 formal case 必须分目录；正式矩阵至少含 nominal hold、正负 pitch 初值、正负 rolling初值、正负 base-X force impulse、正负 pitch moment impulse、对称 leg posture perturbation、torque saturation、contact-loss/invalid-state fail-closed 和 reset replay。
- 正式 REVIEW 同时检查 `theta/dtheta/x/dx/z`、joint errors、六路 torque decomposition、左右 wheel equality、contact continuity、normal force symmetry、slip/penetration、closure、roll/yaw/Y泄漏和 recovery time；平均值不能掩盖瞬时 fall/contact loss。
- standing PASS 至少要求一个 `10 s` nominal hold 和所有冻结小扰动 case 通过；fall、非预期 contact、单轮持续离地、NaN/Inf、越限或未恢复均为失败。
- 检查 default zero、Phase 17 Joint PD、Phase 18 contact plant 与历史 evidence 未覆盖；future SolidWorks/identified profile 必须通过新 equilibrium/local-model/gain/config/run 追加复现。

## Acceptance Criteria

- [ ] T01–T11 完成，DG01–DG09 全部关闭且没有未记录偏差。
- [ ] current nominal contact-aware equilibrium 有完整 provenance，并通过双轮 contact、闭链、静态残差、左右载荷/torque 对称和 reset oracle。
- [ ] `theta/dtheta/x/dx` 的 frame、单位、正方向、rolling关系和 reset anchor 经独立 MuJoCo/finite-difference oracle 验证；公共 RobotState schema 不扩张，Core 不复制 wheel-center kinematics。
- [ ] standing mode 严格 opt-in；默认配置仍零输出。腿部使用冻结的 gravity+support+PD，轮子只使用 equal common-mode balance，任何 differential torque 为零。
- [ ] 4-state/1-input 10 ms local model、gain 和有效 envelope 被版本化并通过 controllability/poles/holdout/full-plant exploratory gates；Core 不依赖 MuJoCo或在线完整动力学。
- [ ] 正式矩阵至少连续站立 `10 s`，且 pitch、height、rolling drift、leg posture、contact、penetration、closure、torque/velocity/saturation、finite 和 roll/yaw/Y泄漏全部满足预冻结阈值。
- [ ] 正负初始偏差和外部 force/moment/posture 扰动均在冻结 recovery window 内恢复；超 envelope、contact loss、非法/nonmonotonic state fail closed 并锁存到 reset。
- [ ] 2 ms physics / 10 ms control / 5-step ZOH、source/receipt time、watchdog、reset顺序、append-only log 和 non-overwrite 保持；fresh-process replay 通过。
- [ ] Phase 02/04/14/15/16/17/18 与 C++/ROS/Adapter tests 全部回归 PASS；历史正式 evidence 未修改。
- [ ] 方法、README、ROADMAP、实现、profile、manifest 和真实 evidence 一致；所有结论明确限定 current nominal、planar、小扰动、simulation-only，不宣称 WBC、真机或大范围抗跌倒。

## Execution Notes

- 2026-08-25：用户要求制定 Phase 19，不开始实现。计划采用固定对称腿姿态 + common-wheel 4-state反馈；完整 MuJoCo 多刚体/contact plant 提供最终证据，但在线 Controller 不求解完整多刚体动力学。
- 2026-08-25：`z` 在本 Phase 不设独立 Cartesian controller，而由 leg posture/support间接维持并作为硬验收指标；显式 height/wrench/contact-force 分配留给后续 Weighted WBC。
- 2026-08-25：Graphify 只查询现有本地图；没有执行 extract/update。live Core/Adapter/runner 与 Simulink候选均由 CBM 和直接源码核对。
- 2026-08-25：用户要求执行 Phase 19，状态转为 `active`；开始关闭 DG02–DG07，不连接真机。
- 2026-08-26：预冻结探索得到可控秩 4 的 10 ms 四状态模型，但候选闭环谱半径 `1.0320567`，且 10 s full-plant cases 存在恢复偏差和横向/姿态泄漏。两次 replay exact、non-overwrite PASS；按 PLAN 停在 Core 实现前并进入 REVIEW=`REWORK`。

## Blockers

当前 blocking：零轮扭矩 contact-aware equilibrium 未关闭；候选四状态反馈闭环谱半径大于 1；完整 3D plant 在不控制 roll/yaw 时泄漏超限，而诊断性平面稳定外力不属于冻结 scope 且仍未通过恢复 gate。需要先决定并重划“显式受约束的 2D sagittal plant”与“完整 3D standing controller”的边界，再重做 DG02–DG05；不得直接进入 Core/formal 或创建 RECORD。
