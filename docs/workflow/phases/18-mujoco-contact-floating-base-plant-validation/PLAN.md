# Phase 18: nominal 轮地接触与 floating-base plant 验证 — PLAN

Status: `complete`

## Goal

在不连接真机、不设计站立控制器的前提下，为 current nominal MuJoCo 模型建立显式、可复用的轮地接触 profile 和 2 ms 物理步诊断，分层验证法向支撑、滚动方向、纵向/横向滑移与摩擦趋势，以及完整机器人 floating-base 的自由落体、触地、base state 和 reset，使 Phase 19 简单站立只新增控制问题，不再同时猜测 contact plant 是否工作。

## Current State

- 已有：Phase 02/04 冻结 canonical FLU、六关节顺序、`q_C=-q_M+b`、`dq_C=-dq_M`、`tau_M=-tau_C`、base COM-site state、命名 wheel-floor 二值 contact 和 floating reset；当前 Adapter 只输出 `contact/no-contact`，不输出接触力。
- 已有：Phase 15 冻结 current nominal 轮轴为世界 `+Y`、名义半径 `0.05 m`、正 canonical wheel 速度对应轮心无滑前滚 `+X`，并验证左右轮心/接触点镜像与 reduced Jacobian。
- 已有：Phase 16/17 提供同一 C++ Controller↔Adapter↔MuJoCo loop、`0.002 s` physics、`0.010 s` control、5-step ZOH、reset/replay/non-overwrite，以及 fixed-base/contact-disabled Joint PD+gravity；这些结果没有验证 contact 或 floating-base 控制。
- 当前 `scence.xml` 依赖隐式 MuJoCo contact defaults：`condim=3`、friction `[1,0.005,0.0001]`、`solref=[0.02,1]`、`solimp=[0.9,0.95,0.001,0.5,2]`、Newton solver、pyramidal cone；这些数值尚未形成 versioned contact profile，也没有真机标定含义。
- 已发现：`wheel_leg.xml` 当前让全部 imported mesh 使用 `contype=1/conaffinity=1`。current nominal reset 经 `mj_forward` 已产生 11 个连杆内部 mesh contact；释放 base 后还会出现非轮子—地面 contact。直接使用该场景不能隔离轮地接触结论。
- 已发现：Phase 16 C++ runner 将 `AdapterConfig::floating_base` 保持默认 `false`，只按 10 ms control tick 保存二值 contact；没有 2 ms impact/contact 行、per-contact force、penetration、system COM/momentum 或 solver-health 诊断。
- Grounding：live code 依据 CBM generation `2026-08-25T06:16:31Z` 与直接源码读取；`deterministic_loop.cpp` 尚未被该 generation 跟踪，`adapter.hpp:28` 有 partial coverage，均已直接核对。Graphify 只查询现有本地图，没有执行 extract/update。

## Scope

- Ground current nominal collision mesh、接触过滤、solver/contact defaults、Adapter contact/reset 和 Phase 15 rolling contract，形成 compiled contact/collision manifest。
- 把 imported visual/inertial mesh 与本 Phase 的 collision eligibility 分开：默认禁用非轮 mesh contact，只让命名 `left_wheel_collision`、`right_wheel_collision` 与 `floor` 进入正式 wheel-ground contact set；质量、COM、惯量和 mesh 资产不因 collision mask 改变。
- 建立显式 `phase18_nominal.json` 和 full-robot contact scene，版本化 timestep、solver、cone、iterations、`condim`、friction、`solref/solimp`、margin/gap、gravity、collision set、initial state、case matrix 和 thresholds。
- 建立最小 actual-wheel-mesh probe fixture：左右轮分别复用 current nominal collision mesh；使用版本化的受限 carriage DOF 与已知 mass/inertia，把 normal、longitudinal rolling 和 lateral slip 从完整闭链/平衡问题中隔离出来。
- 实现验证侧 per-contact 诊断：接触对、接触点、法向、distance/penetration、维数、作用在机器人/轮上的 world-FLU force/torque、轮心/接触点速度和 slip；用 geom 名称处理 pair 顺序，不假设 `geom1/geom2` 固定。
- 兼容扩展 Phase 16 C++ loop，而不复制第二套 plant step：显式选择 fixed/floating、初始 base pose/twist、Controller mode、外部 wrench/impulse、contact diagnostics，并追加 2 ms physics-step 日志；10 ms control-tick 基础列与语义保持不变。
- 以 frictionless/nominal/high-friction 对照验证趋势，以 probe 的重量平衡与冲量—动量平衡验证法向力数量级；这些 sweep 是内部 oracle，不是三个已标定真机 surface profile。
- 在完整 nominal 双腿模型执行 zero-command free flight/touchdown，以及必要的短时、对称、受限激励；验证 system COM、base state、wheel-floor contact、约束、有限值、solver health 和 reset replay，不把保持直立时长作为 PASS 指标。
- 回归 Phase 02/04/14/15/16/17，固化 model/contact/controller/runner/config/output hashes、完整 raw rows、失败样本和跨 revision/identified profile 的非覆盖入口。

## Out of Scope

- 真机上电、STM32/树莓派联调、Load Cell、轮胎实验、地面摩擦测量、接触辨识或任何 MuJoCo–real 一致性结论。
- base height/pitch、wheel position、平衡、站立、抗跌倒、WBC、NMPC、trajectory/planner 或 Phase 17 gain retune；Phase 19 才开始简单站立。
- 宣称当前 CAD wheel mesh、`mu=1`、contact compliance、restitution 或 solver 参数具有真实轮胎/地面精度；本 Phase PASS 仅说明冻结 MuJoCo profile 内部一致、方向正确和数值可用。
- 轮胎弹性体、deformable contact、轮胎刷子模型、滚动阻力辨识、地形/台阶、左右不同地面或高速冲击。
- 为验证工具扩张 `RobotState`/`TorqueCommand`/ROS message schema；接触力、penetration、slip 和 system COM 只写 validation diagnostics。
- 把所有机器人部件都建成高保真 collision shapes。非轮部件触地/碰撞安全模型若后续需要，必须单独设计；不能重新启用当前全部 CAD component mesh 碰撞。
- 修改 canonical frame/sign/order、Phase 15 名义轮半径、Phase 16 timing、Phase 17 控制律或历史正式 evidence。

## Frozen Decisions

- authoritative plant 仍是 current nominal `wheel_leg.xml`；Phase 18 只增加显式 collision eligibility/contact scene/profile。后续 SolidWorks 导出、identified plant 或 contact calibration 使用新 revision/profile/run，不覆盖 nominal 文件与证据。
- physics/control timing 固定为 `0.002 s / 0.010 s / 5-step ZOH`。impact/contact verdict 使用每个 2 ms physics step 的 post-step 行；Controller 仍只在 10 ms tick 更新，不能用 control-rate 日志掩盖触地瞬态。
- 正式 wheel-ground set 仅为 `{floor,left_wheel_collision,right_wheel_collision}` 的两个命名 pair。全部普通 imported component mesh 使用 `contype=0/conaffinity=0`，floor 使用 `contype=1/conaffinity=0`，左右 wheel collision geoms 使用 `contype=0/conaffinity=1`；按 MuJoCo bitmask 只允许 floor–wheel，不允许 wheel–wheel、floor–floor 或 wheel–普通 mesh。reset、自由飞行和正式窗口内出现未批准 contact pair 直接失败。
- nominal contact profile 显式保存当前 compiled baseline：Newton solver、pyramidal cone、`100` iterations、`50` line-search iterations、`condim=3`、friction `[1,0.005,0.0001]`、`solref=[0.02,1]`、`solimp=[0.9,0.95,0.001,0.5,2]`、zero margin/gap。由于 `condim=3`，正式结论只覆盖法向和二维 sliding friction；不声称 torsional/rolling-friction 参数已验证。
- probe fixture 必须使用左右实际 wheel collision mesh，而不是用理想圆柱替代。carriage 的约束、mass/inertia 和 excitation 全部进入 manifest；probe 只给接触原语的方向、守恒和趋势证据，不代表完整机器人性能。
- 法向/力统一定义为“floor 作用在 wheel/robot 上”的 world-FLU wrench，正常支撑的 `Fz>0`。per-contact frame 转换必须同时通过静态重量平衡、冲量—动量或等价独立 oracle，不能只自测同一转换函数。
- rolling residual 服从 Phase 15 契约：正 canonical wheel speed 的无滑轮心方向为 `+X`，名义关系为 `v_center,x-r*dq_C≈0`；正式测试必须同时覆盖左右轮、正负方向和零/高摩擦对照。
- lateral 定义为 world `Y`，normal 为 world `Z`。摩擦验证以 compiled cone 对应的不等式、切向摩擦功率非正、冲击/回弹总能量有界，以及 `mu=0 < nominal < high` 的滑移/减速趋势为 gate；不拿一次动画或最终位移单独判定。
- full-robot floating verdict 分离为：pre-contact free flight、touchdown/contact interval、post-contact bounded window。zero-command free flight/touchdown 是 plant authority；若使用 Phase 17 Joint PD+gravity 维持短时 leg posture，必须标为 test fixture，单独报告且不能形成站立 PASS。
- full-robot Newton–Euler/impulse gate 使用全系统 COM 和总动量；`RobotState.base_position_n_m` 仍是 torso `base_control_frame`，不得误当整机 COM。
- reset 顺序继续是 simulation/Adapter 后 Controller。contact warm-start、外部 wrench、旧 command、contact rows 和 quaternion history 都不得跨 episode；相同 frozen input 的重复 episode/run 必须在预冻结容差内重放。
- exploratory solver/threshold runs 与 formal holdout 分目录；formal config/hash 生成后不得放宽 threshold 或删除失败 case。必要参数变更只能追加新 profile/run，并在 REVIEW 解释原因与影响。
- Phase 18 的零控制 contact plant 使用 MuJoCo 3.7.0 Python binding 作为唯一正式 2 ms physics loop；它不经过或复制 Controller↔Adapter 调度。Phase 16/17 C++ loop 原样保留并通过回归证明未受影响；Python runner同时负责编排、contact oracle、逐步日志、汇总和 manifest。

## Open Questions / Decision Gates

- **DG01 / CLOSED / CODEX_DECISION — Phase 边界：** 本 Phase 只验证 contact plant 与 floating-base state/reset，不设计站立；Phase 19 才使用已验证 plant 做 z/pitch/leg posture/wheel position 控制。
- **DG02 / CLOSED / EVIDENCE — collision set：** wheel-only mask 后 compiled model 无初始自碰撞、无未批准 pair，左右 actual wheel mesh 的 radius/axis/name 与 Phase 15 manifest 一致。
- **DG03 / CLOSED / EVIDENCE — force/frame oracle：** geom pair 顺序、contact frame 到 world-FLU 的作用/反作用符号、per-wheel aggregation 已通过独立重量与冲量/动量检查。
- **DG04 / CLOSED / EVIDENCE — numerical envelope：** exploratory runs 冻结了 penetration、force/impulse、friction power、constraint、acceleration 和 symmetry thresholds；formal-v3 全部通过。
- **DG05 / CLOSED / EVIDENCE — contact primitive：** 左右 wheel probe 的 normal settle/drop、正负 rolling/lateral、frictionless/nominal/high-friction matrix 全部通过方向、守恒、friction-cone/power 和趋势 gate。
- **DG06 / CLOSED / EVIDENCE — integrated floating plant：** 完整模型的 free flight、touchdown、base pose/twist/quaternion、system COM、closure residual、有限值和 reset replay 通过冻结 `0.2 s` 窗口；不要求站立。
- **DG07 / CLOSED / EVIDENCE — compatibility/reuse：** Phase 02/04/14/15/16/17 回归、fresh-process replay 和 non-overwrite 全部 PASS；新 profile 可只换 config/new run 重跑。
- **DG08 / CLOSED / SCOPE — physical fidelity：** 没有真实接触实验时只能关闭 nominal MuJoCo internal-consistency gate；真实 friction/compliance/rolling resistance 在真机解冻后的共同辨识 Phase 关闭。

## Interfaces and Compatibility

- 输入：current nominal model；`phase18_floating_contact.xml` / `phase18_wheel_contact_probe.xml`；versioned contact config；canonical `RobotState/TorqueCommand`；可选 Phase 17 frozen controller profile；initial base pose/twist、wheel torque、carriage velocity 或 external wrench schedule。
- control-tick 输出：保持 Phase 16/17 CSV 基础字段，并追加 selected plant/contact summary；`source_time_ns` 仍来自 `mjData.time`，receipt clock 仍只服务 watchdog。
- physics-step 输出：episode/case/step/time、base site state、system COM/momentum、closure residual、每轮中心/角速度/接触点速度/slip、contact count、per-wheel world wrench、min distance/max penetration、solver/finite flags；per-contact 明细以稳定 geom names 关联。
- manifest：profile/model revision、scene/model/controller/config/runner hashes、MuJoCo 版本、solver/contact 参数、collision mask、initial state、case matrix、seed、thresholds、schema version、run ID、`supersedes`、hardware-data=false。
- 必须保持：Core 无 MuJoCo/ROS 依赖；公共 message 不变；Adapter sign/order/watchdog 和二值 contact 语义不变；Phase 15 rolling contract、Phase 16 timing/reset/ZOH、Phase 17 default-zero 和 controller diagnostics 不变。
- 允许改变：为 imported geoms 增加显式 collision mask/class；新增 Phase 18 scenes/config/probe、Python validation runner、tests、方法和 evidence。Phase 16 C++ runner与公共接口不修改。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| T01 | Ground current collision/contact/solver 与 Phase 15–17 契约 | CBM/source、compiled `scence.xml`、Phase 04/15/16/17 evidence、现有 Graphify 图 | grounding、compiled collision/contact manifest、职责与增量边界 | 记录全部 eligible pair、初始 11 个自碰撞、wheel radius/axis、defaults、Adapter/runner 缺口；不执行 Graphify 更新 | done |
| T02 | 建立 wheel-only collision eligibility 并关闭 DG02 | T01、`wheel_leg.xml`、Phase 15 geometry manifest | 最小 collision class/mask、`phase18_floating_contact.xml`、model invariant tests | 质量/COM/inertia/hash 影响可解释；无初始/内部未批准 pair；左右仅能与命名 floor 接触；Phase 15 geometry 回归 PASS | done |
| T03 | 冻结 nominal contact profile 与 case/threshold protocol | T01/T02、compiled defaults、MuJoCo 3.7.0 | `phase18_nominal.json`、exploratory/formal split、case IDs、threshold schema | 所有 solver/contact/init/excitation/threshold 字段显式且有单位；formal hash 在 holdout 前冻结 | done |
| T04 | 实现 world wrench/slip/system diagnostics，关闭 DG03 | T02/T03、MuJoCo contact API、Phase 15 direction contract | Python 2 ms per-wheel/contact aggregation schema 与 oracle | pair 顺序归一；静态重量与冲量—动量独立 oracle 过阈值；world FLU/作用对象可审计 | done |
| T05 | 建立左右 actual-wheel probe 与唯一 Phase 18 Python physics loop | T02–T04、MuJoCo 3.7.0 binding | `phase18_wheel_contact_probe.xml`、floating/contact runner 与逐 2 ms CSV；Phase 16 loop不改 | 左右 mesh/axis/radius 与主模型一致；runner 拒绝覆盖输出；所有 case 使用同一 step 实现 | done |
| T06 | 执行 exploratory numerical envelope 并关闭 DG04 | T03–T05、drop/speed/torque/friction sweep | solver health、penetration、force/impulse、energy、constraint、symmetry envelope 与 frozen formal thresholds | 探索 run 全保留；无 warning/NaN/Inf/非物理主动注能；失败时修 profile 或 REWORK，不删 case | done |
| T07 | 正式验证 wheel contact primitive 并关闭 DG05 | frozen profile/thresholds、T04–T06 | 左右 normal settle/drop、±rolling、±lateral、friction sweep raw/summary | 重量与冲量平衡、方向、cone/power、slip trend、左右 symmetry 全部 PASS | done |
| T08 | 正式验证 full-model floating plant 并关闭 DG06 | T02–T07、current nominal full model、zero command | free-flight/touchdown/bounded-contact/reset/replay raw/summary | pre-contact gravity、wheel-only contact、base pose/twist、COM、closure、finite 和 replay 全部 PASS；不评价站立 | done |
| T09 | 执行历史回归与跨 profile 复用检查，关闭 DG07 | T02–T08、Phase 02/04/14/15/16/17 entrances | colcon/coordinate/dynamics/kinematics/loop/PD regression、新 profile dry-run、non-overwrite evidence | 旧 tests/runner/schema 全部 PASS；历史 evidence 未修改；换 config/new run 可执行 | done |
| T10 | 固化方法、grounding、reuse contract、正式 evidence 并准备 REVIEW | T01–T09 | `run_mujoco_contact_floating_base.py`、方法文档、README、automated evidence、Execution Notes、REVIEW 输入 | DG01–DG08 全关闭；manifest/raw/summary 可复现；所有结论明确限定 nominal simulation-only | done |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Automated

- `cd ros_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-up-to wheel_leg_mujoco`：扩展后的 Core/ROS/Adapter/runner 在 C++17、ROS2 Jazzy、MuJoCo 3.7.0 下构建。
- `cd ros_ws && source /opt/ros/jazzy/setup.bash && colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco && colcon test-result --verbose`：collision invariant、contact force/frame、Adapter bits/reset 和既有 Core/ROS/Adapter tests 无失败。
- `./.venv/bin/python tools/experiments/run_mujoco_contact_floating_base.py --config simulation/mujoco/config/phase18_nominal.json --output-dir data/experiments/<new-phase18-run-id>/raw`：执行 frozen probe + full-model formal matrix，所有 DG02–DG06 gates PASS。
- 对相同 frozen config 使用两个新的 output directory：11 个 physics-step CSV 在 determinism tolerance 内一致；再次指向非空目录必须在仿真前失败。
- `./.venv/bin/python tools/maintenance/test_mujoco_coordinate_contract.py`：FLU、joint sign/order、wheel axis/rolling direction 回归 PASS。
- `./.venv/bin/python tools/experiments/run_mujoco_internal_dynamics.py --output-dir data/experiments/<new-phase14-regression-id>/raw`：Phase 14 contact-free 内部动力学回归 PASS。
- `./.venv/bin/python tools/experiments/run_mujoco_closed_chain_kinematics.py --output-dir data/experiments/<new-phase15-regression-id>/raw`：Phase 15 radius/contact point/reduced Jacobian 回归 PASS。
- `./.venv/bin/python tools/experiments/run_mujoco_controller_loop.py --profile nominal --output-dir data/experiments/<new-phase16-regression-id>/raw` 与 `./.venv/bin/python tools/experiments/run_mujoco_joint_pd_gravity.py --config simulation/mujoco/config/phase17_nominal.json --output-dir data/experiments/<new-phase17-regression-id>/raw`：旧 timing/fault/reset/PD+gravity 结果继续通过且可解析追加后的 runner schema。

### Manual / Evidence

- Codex 在正式 run 前审查 collision eligibility：对每个 compiled geom 记录 name/body/class/contype/conaffinity；正式 pair 集合外任一 active pair 都阻断执行。
- 接触力 REVIEW 同时检查 raw contact-frame 值、world-FLU 聚合、pair 顺序、作用对象、静态重量残差、冲量—动量残差和接触功率；不能只看汇总 PASS 布尔值。
- formal matrix 至少包含左右轮、normal settle/drop、正负 wheel torque、正负 longitudinal/lateral initial velocity、frictionless/nominal/high-friction、zero-command full-model touchdown、contact 后 reset 和跨进程 replay。
- 检查 floating RobotState 的 quaternion norm/sign continuity、base COM-site pose/twist 与直接 MuJoCo oracle；system COM/momentum 另列，不混用两个 origin。
- 可选 Phase 17 posture fixture 的行、时间窗和 command 必须显式标记；任何“站了多久/是否摔倒”现象只能记观察，不进入 Phase 18 PASS。
- evidence 保存完整 2 ms per-wheel aggregate rows、失败 case、manifest 和 hashes；formal evidence/旧 Phase evidence 不原地覆盖。

## Acceptance Criteria

- [x] T01–T10 完成，DG01–DG08 全部关闭且没有未记录偏差。
- [x] current nominal compiled model 只有命名左右 wheel-floor pair 可进入正式 contact set；初始自碰撞和未批准 floor/body contact 为零，mass/COM/inertia 与 Phase 14 nominal 基线未被 collision mask 意外改变。
- [x] nominal solver/contact/collision/initial-state profile 全部显式版本化；2 ms physics-step contact 诊断可追溯，Phase 16/17 control log 与 runner 未修改。
- [x] wheel contact wrench 正确归一化为 floor 对 wheel 的 world-FLU 作用；静态重量和冲量—动量独立 oracle 通过预冻结阈值。
- [x] 左右 actual-wheel probe 的 normal、正负 rolling、正负 lateral、friction sweep 通过 direction、cone/power、slip trend、symmetry、finite 和 numerical-envelope gates。
- [x] full-model zero-command free flight/touchdown 在冻结窗口通过 gravity、system COM、wheel contact、base pose/twist/quaternion、closure、finite 和 reset replay gates；不要求或宣称站立。
- [x] Adapter 命名 wheel-floor 二值 contact 单元回归 PASS；未向公共 `RobotState`/ROS schema添加 simulator-only force 字段。
- [x] Phase 02/04/14/15/16/17 自动回归 PASS；历史 evidence 未覆盖，新 model/contact profile 可通过 config/new run 使用同一入口重跑。
- [x] REVIEW 结论明确限定为 current nominal MuJoCo internal consistency；真实 friction/compliance/rolling resistance、真机接触与站立仍为后续 gate。

## Execution Notes

- 2026-08-25：用户要求制定 Phase 18，本次只创建 PLAN 和路线链接，不开始实现或仿真正式执行。
- 2026-08-25：采用 ponytail 的最小分层设计：一个 actual-wheel probe 隔离 contact primitive，一个 full-model scene 验证 integrated floating plant；不增加站立控制或公共消息。
- 2026-08-25：现有 Graphify 图只用于核对 Phase 04/14/15/16 历史关系，没有执行 extract/update；live facts 以 CBM/source 和本机 MuJoCo 3.7.0 compiled probe 为准。
- 2026-08-25：compiled `scence.xml` 在 reset 已有 11 个内部 CAD mesh contact，当前所有 imported geoms 都可碰撞；该 finding 已登记为 T02/DG02，wheel-only collision eligibility 未通过前禁止动态 contact 放行。
- 2026-08-25：Phase 18 不把 contact defaults 称为标定参数。`mu=1` 等只冻结为第一轮 nominal simulation profile，未来 MuJoCo–real contact identification 必须创建 identified profile 并追加重跑。
- 2026-08-25：执行时将原定“扩展 Phase 16 C++ loop”改为独立的零控制 MuJoCo Python plant runner。原因是 Phase 18 不经过 Controller/Adapter 调度；保持 Phase 16 runner 不变可减少共享回归面。该变更未改变 2 ms timestep、contact authority 或公共接口。
- 2026-08-25：`formal-v5` 为正式 authority，20/20 gates PASS；11 个 CSV 跨进程逐字节一致，非空目录拒绝覆盖。Phase 02/04/14/15/16/17 回归全部 PASS。
- 2026-08-25：REVIEW 无 blocking finding，Verdict `PASS`；创建 RECORD 并将 Phase 状态更新为 `complete`。

## Blockers

None.
