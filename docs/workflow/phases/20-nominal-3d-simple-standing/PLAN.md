# Phase 20: nominal 完整 3D 简单站立 — PLAN

Status: `complete`

## Goal

在不连接真机、不引入 WBC/QP/NMPC 的前提下，直接使用 current nominal 完整 3D MuJoCo floating/contact plant，在 canonical C++ Controller Core 中实现一个显式 opt-in 的简单站立模式：保持双轮接触和固定腿姿态，以 common wheel authority 稳定 `X/pitch`、以 differential wheel authority 保持 heading、以差分腿力矩 authority 稳定 roll，并使未直接控制的 `Y/Z`、接触、闭链、滑移和六路力矩在冻结小扰动范围内有界；全部结论仅限 current nominal simulation-only。

## Current State

- 已有：[Phase 18](../18-mujoco-contact-floating-base-plant-validation/RECORD.md) 已验证 current nominal `wheel_leg.xml`、`phase18_floating_contact.xml`、wheel-only contact、完整 freejoint base state/reset 和零控制 touchdown，但只覆盖短时 plant 一致性，不是站立证据。
- 已有：[Phase 19](../19-nominal-planar-simple-standing/RECORD.md) 已在 derived exact-planar plant 上通过 fixed-leg + common-wheel 简单站立、`2 ms / 10 ms / 5-step ZOH`、fault/reset/replay 和非覆盖 formal；其模型删除了 `Y/roll/yaw`，数值 equilibrium、support、gain 和 threshold 不能直接复用。
- 已有：公共 `RobotState` 已提供 `base_position_n_m[3]`、完整 quaternion、world linear/angular velocity、六关节状态和左右 contact；`TorqueCommand` 已提供六路 canonical joint torque。Phase 20 不需要改变公共消息 schema。
- 已有：MuJoCo Adapter 已从 `base_control_frame` 输出完整 3D pose/twist，floating reset 会关闭 `base_weld`，并保持 canonical joint/order/sign、watchdog 和 fail-to-zero。
- 缺少：完整 3D 双轮 contact/equality equilibrium、admissible 3D perturbation、world-axis orientation error、common/differential wheel 与 roll-leg 三路可实现输入基、3D sampled local model、完整非线性 pre-freeze authority、独立 Core mode/runner 和 3D formal matrix。
- 失败证据：Phase 19 v1 在完整 3D plant 上只使用四状态/common-wheel controller 时，nominal 10 s 出现约 `0.0544 m` lateral 漂移和 `0.1312 rad` roll/yaw 泄漏；这证明原控制 authority 不足，不证明本 Phase 冻结的新结构必然可行。
- 历史图：执行开始时Graphify缺少Phase 20 active关系；维护代理已完成最小增量刷新。实现与验收仍以live source、CBM generation `2026-08-26T07:29:33Z` 和真实evidence为准。

## Scope

- 以 `simulation/mujoco/model/phase18_floating_contact.xml` 及其包含的 authoritative `wheel_leg.xml` 为唯一完整 3D plant；运行时关闭 `base_weld`，保留六自由度 freejoint、左右闭链、六 actuator、wheel-only contact、solver/contact/timestep，不派生 planar 模型、不增加隐藏约束或虚拟 lateral force。
- 重新求解并验证完整 3D、upright、双轮接触、零速度、zero-wheel-torque equilibrium；world `X/Y/yaw` 只作为 gauge 固定，leg reference/support、base height、左右 normal load 和 closure 必须由本 plant 重新得到。
- 建立 contact/equality-consistent 的 reset 与 perturbation 生成器；raw freejoint/joint `qpos/qvel` 扰动若离开 admissible manifold，只能标为 diagnostic，不能作为 gain 或 release oracle。
- 冻结 3D state/sign/reset contract：world FLU、quaternion shortest-arc orientation error、world angular velocity、position/heading anchor、左右 contact、common/differential wheel sign 和 roll-leg input sign均有正负 oracle。
- 建立三路虚拟输入 `u=[u_common, u_roll, u_yaw]`：common wheel 负责 `X/pitch`，zero-wheel-entry 的 leg torque direction 负责 roll，differential wheel 负责 heading；用 10 ms sampled full nonlinear contact plant 生成可审计的局部模型和静态 state-feedback gain。
- 在任何 Core 修改前执行 pre-freeze：证明输入 authority、数值差分/辨识收敛、闭环局部稳定性、正负 holdout、cross-coupling 和完整非线性 10 s recovery；失败即 REVIEW=`REWORK`，不以调大阈值或增加隐藏外力继续实现。
- 为 Controller Core 新增独立 opt-in `kSimpleStanding3d`（最终命名在实现中保持语义等价）和 additive diagnostics；Phase 19 `kSimpleStanding`、Phase 17 `kJointPdGravity`、default-zero、公共消息与 Adapter 边界保持不变。
- 新增独立 full-3D C++ standing loop；复用 Adapter、双时钟、watchdog、2 ms physics、10 ms control、5-step ZOH、reset/replay 和 non-overwrite，不把 `planar_standing_loop` 改名或当作 3D authority。
- 建立正式方法、versioned equilibrium/prefreeze/formal profile、Python orchestration/evaluator、逐 control-tick 与必要 physics-row 日志、summary/manifest/hash，并执行独立 3D formal、fresh replay、历史回归和 revision reuse dry-run。

## Out of Scope

- 真机上电、吊架/保护架实验、STM32/树莓派联调、传感器/执行器/接触辨识，以及任何 MuJoCo–real 一致性或安全结论。
- absolute world `Y` position regulation、lateral trajectory、yaw-rate tracking、turning、continuous turning、单轮支撑、斜坡、台阶、跌倒恢复或大范围 region-of-attraction 声明。
- 显式 Cartesian height/wrench/contact-force controller、inverse dynamics、QP、Weighted WBC、NMPC、积分器、observer、gain scheduling、在线 MuJoCo linearization 或 runtime MuJoCo dynamics oracle。
- 用 lateral spring/damper、penalty stabilizer、隐藏 weld、每步覆写 state、非物理 contact、阈值放宽或外部辅助力伪造 3D standing。
- 修改 canonical FLU/frame/sign/order、公共 `RobotState/TorqueCommand`/ROS schema、Phase 18 contact profile、Phase 16 timing、Phase 19 planar mode或任何历史正式 evidence。

## Frozen Decisions

- **Plant authority：** Phase 20 直接使用 current nominal full-3D `phase18_floating_contact.xml`；`base_weld` 在 floating reset 后必须 inactive。manifest 必须记录 source model、scene、mesh、MuJoCo、solver/contact 和 compiled dimensions/hash。
- **No 2D carry-over：** Phase 19 的 exact-planar equilibrium、side-specific reference/support、`8/1` leg gain、四状态 gain 和 formal thresholds 只可作为 seed/比较项，不能成为 Phase 20 authority。
- **Timing：** physics `0.002 s`，control `0.010 s`，5-step ZOH；Controller 只在 control tick 更新，六路 torque 在随后五个 physics step 完全不变。
- **Equilibrium：** target 为 world-up upright（roll/pitch 为零）、双轮 contact、零速度、左右 wheel torque 为零；`X/Y/yaw` 固定为 gauge。若 current nominal 不能在冻结 residual/contact/torque 条件下得到该 equilibrium，本 Phase REWORK，不用倾斜地面、隐藏外力或 controller steady bias 替代。
- **Orientation state：** 令 `R_n_b` 为 `q_n_from_b` 对应旋转，`R_ref` 为 world-up 且采用 reset 首帧 heading 的参考；orientation error 固定为 world-axis shortest-arc `e_R = Log(R_n_b R_ref^T)`，有效 envelope 小于 `pi`。角速度直接使用 `base_angular_velocity_n_rad_s`。
- **Feedback state order：** `x_8=[x-x_0, v_x, e_R_y, omega_y, e_R_x, omega_x, e_R_z, omega_z]`，依次对应 `X/pitch`、roll、heading。`y-y_0/v_y/z-z_0/v_z` 被完整 sensing、logging 和 safety gate 覆盖，但不设置 absolute `Y` position task；无侧向执行器的 stationary nonholonomic 约束不得被伪装成可独立控制自由度。
- **Virtual inputs：** canonical joint order 为 `[left_hip,left_knee,left_wheel,right_hip,right_knee,right_wheel]`。`tau_left_wheel=u_common+u_yaw`，`tau_right_wheel=u_common-u_yaw`；`u_roll` 通过一个 versioned、wheel entries 为零、单位范数且正号产生 world `+X` roll acceleration 的 leg direction `s_roll` 注入。`s_roll` 的数值由 DG20-03 证据冻结，不能凭对称外观猜测。
- **Controller：** leg baseline 为本 Phase equilibrium 的 side-specific support + fixed-reference sampled PD；outer command 为无积分、无在线求解的静态反馈 `u=-K_3d x_8`。`K_3d`、`s_roll`、A/B 或等价辨识模型、设计输入、poles、stabilizability、residual 和有效 envelope 全部进入 profile/manifest；runtime Core 不链接 MuJoCo。
- **Lateral/height semantics：** `Y` 依靠真实 wheel-floor lateral contact/friction保持，`Z` 依靠 equilibrium support与固定腿姿态间接保持；两者必须在正负扰动下有限、速度衰减且不失去 bilateral contact。若证据不支持，必须 REWORK 到新的控制架构 Phase，不在本 Phase 偷加 Cartesian force task。
- **Heading semantics：** 本 Phase保持 reset 时的 heading，并验证 yaw moment/yaw-rate 扰动恢复；不声明 absolute compass heading、yaw-rate tracking 或 turning。
- **Pre-freeze first：** equilibrium、state/input sign、admissible perturbation、authority/stabilizability 和 full nonlinear holdouts 全 PASS 后才允许修改 Core/C++ formal chain。raw full-coordinate finite-difference poles不作为唯一 physical authority。
- **Fail closed：** invalid quaternion/state、non-monotonic/timing error、任一 wheel contact loss、position/orientation/height/leg/joint-velocity envelope、non-finite command或任一路 torque saturation均输出六路零并锁存到 reset。
- **Evidence freeze：** exploratory 与 formal 使用不同新目录；formal case、threshold、profile、binary/source/config hash 在运行前冻结，formal 失败后不得原地调参重跑，必须新增 run/profile 并记录 supersedes。
- **Claims：** PASS 只表示 current nominal full-3D MuJoCo 在冻结小扰动矩阵中的简单站立；不表示真机、identified profile、WBC、NMPC、turning 或鲁棒稳定性已通过。

## Open Questions / Decision Gates

- **DG20-00 / CLOSED / CODEX — route：** 保持完整 3D plant，新增 roll 与 heading authority；`Y/Z` 为真实 plant outcome/safety，不提前进入 Cartesian WBC。
- **DG20-01 / CLOSED / EVIDENCE — full-3D equilibrium：** v1 equilibrium 与 fresh replay exact hash一致；最大qacc `2.45e-11`、generalized residual `1.51e-10`、closure `1.63e-4 m`、左右normal load `30.96/32.16 N`、one-step qvel drift `4.90e-14`，wheel torque为零。
- **DG20-02 / CLOSED / CODE+EVIDENCE — state/reset/sign：** full-3D compiled/freejoint、orientation Log三轴、world twist、common/differential wheel和roll-leg正负oracle全部通过。
- **DG20-03 / CLOSED / EVIDENCE — realizable authority：** 三路canonical input rank为3、condition number `2.014`；冻结的单位范数 `s_roll=[0.02747397,-0.71819684,0,-0.02705645,0.69477077,0]` 对pitch/yaw cross ratio为 `1.47e-15`。
- **DG20-04 / CLOSED / EVIDENCE — sampled local controller：** v5使用冻结scale的中心差分10 ms transition；8-state controllability rank为8，training/independent-validation normalized RMS为`0.0437/0.0396`，冻结gain闭环谱半径`0.9910`。
- **DG20-05 / CLOSED / EVIDENCE — frozen formal envelope：** v5所有正向tuning与9个未参与选择的负向/组合holdout均在10 s nonlinear plant通过；双轮接触率1.0、torque saturation为零。formal沿用或收紧该profile，不放宽门槛。
- **DG20-06 / CLOSED / CODE+TEST — runtime contract：** additive `kSimpleStanding3d`、x8 quaternion Log、三路torque decomposition、strict contact/timing/fault latch、独立C++ loop、5-step ZOH、reset与旧mode测试全部通过。
- **DG20-07 / CLOSED / EVIDENCE — formal/reuse：** formal-v3的19个10 s normal/perturbation cases和6个双episode fault cases全部PASS；26个fresh replay文件exact，non-overwrite、历史回归和fresh-namespace reuse dry-run通过。
- **DG20-08 / CLOSED / REVIEW — claims：** REVIEW确认无真机/WBC/turning/absolute-Y等越界结论，blocking findings为零，Verdict=`PASS`。

## Interfaces and Compatibility

- 输入：canonical `RobotState`；full-3D Phase 18 scene；versioned equilibrium/prefreeze/formal profile；2 ms world-frame external wrench schedule；10 ms control tick。
- 输出：canonical `TorqueCommand` 六路 N·m；additive `StepResult` 3D state、virtual input、support/PD/balance/raw/saturated/latch diagnostics；runner CSV/summary/manifest/evidence。
- 必须保持：joint order/sign/unit、`q_n_from_b` 和 world twist语义、Adapter watchdog/fail-zero、`0.002/0.010/5` timing、Phase 17/19 mode行为、ROS消息、历史 evidence 与 non-overwrite。
- 允许改变：`ControllerMode/ControllerConfig/StepResult/ControllerCore` 增加独立 3D standing字段；新增 `standing_3d_loop` target、Core/Adapter tests、Phase 20 config、实验方法/工具和文档。
- 不需要改变：`wheel_leg_core/types.hpp` 的公共 `RobotState/TorqueCommand`；Adapter 的正常 pose/twist/contact输出。若实现发现必须改变公共 schema或 plant/contact profile，停止并新开技术决策，不在本 Phase 隐式扩张。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| P20-T01 | 固化 full-3D plant grounding 与 reuse contract | Phase 18 scene/model/profile、Phase 19 reuse | compiled invariant audit、source/scene/mesh/solver/contact manifest、Phase 20 reuse contract | base freejoint完整、base_weld runtime inactive、Phase 18 preserved字段差异为零 | done |
| P20-T02 | 求解 full-3D equilibrium 与 admissible reset/perturbation | P20-T01、Phase 15 closure、Phase 18 contact | `phase20_equilibrium.json`、solver trace、projection residual与 replay | DG20-01；zero wheel torque、upright、bilateral load、qacc/closure/drift/finite PASS | done |
| P20-T03 | 冻结 3D state、heading anchor与 virtual-input sign | P20-T02、RobotState、Adapter、Phase 02/15 sign | orientation/site/twist evaluator、common/differential wheel与 roll-leg正负 oracle、logging schema | DG20-02；analytic/site/Jacobian/finite difference/impulse方向一致 | done |
| P20-T04 | 识别 roll-leg basis 与 10 ms local model | P20-T02/T03、sampled leg PD候选、三路 input | versioned `s_roll/A/B` 或等价 model、authority/stabilizability/fit报告 | DG20-03；步长收敛、目标模态 authority、cross-coupling和torque/contact margin PASS | done |
| P20-T05 | 设计 gain并执行 nonlinear pre-freeze | P20-T04、独立 tuning/holdout matrix | `K_3d`、design inputs、poles/residual、10 s nonlinear summary、frozen formal profile | DG20-04/05；正负单轴与组合 holdout PASS 后才准入 Core | done |
| P20-T06 | 实现 additive 3D Core mode 与单元测试 | 冻结 state/input/config/gain、现有 Core | `kSimpleStanding3d`、config/state/input/torque diagnostics、fail-closed latch/reset tests | DG20-06；三路输入分解、sign、saturation/contact/timing与旧 mode回归 PASS | done |
| P20-T07 | 实现 full-3D C++ loop 与 runtime日志 | P20-T06、Adapter、Phase 16/19 loop contract | `standing_3d_loop.cpp`、CMake target、3D disturbance/fault/reset/ZOH CSV | C++ loop直接调用 Core↔Adapter；无Python controller；5-step ZOH与双时钟 PASS | done |
| P20-T08 | 建立正式方法、wrapper与case matrix | P20-T05/T07、冻结 profile | `docs/experiments/mujoco_3d_simple_standing_validation.md`、Phase 20 config/wrapper/evaluator/manifest | profile驱动、formal前freeze、non-empty目录仿真前拒绝、schema/hash完整 | done |
| P20-T09 | 执行 full-3D formal 与fault matrix | P20-T08 frozen inputs | 新 `evidence/automated/<run-id>/` raw/summary/manifest/validation | DG20-07；全部 normal/perturbation ≥10 s，fault fail-zero/latch/reset PASS | done |
| P20-T10 | fresh replay、历史回归与revision reuse audit | P20-T09、Phase 02/14–19入口 | replay comparison、colcon/coordinate/plant/controller regressions、fresh namespace dry-run | deterministic/容差一致；旧 evidence未覆盖；reuse pipeline贯通 | done |
| P20-T11 | REVIEW | 全部任务与真实 evidence | `REVIEW.md`；仅 Verdict=`PASS` 后创建 `RECORD.md` | DG20-08关闭，blocking findings=0，ROADMAP随后才可 complete | done |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Pre-freeze

- plant/equilibrium：compiled full-3D dimensions、freejoint/base-weld状态、wheel-only contact、solver/contact options、upright zero-wheel-torque equilibrium、qacc/generalized residual、closure、normal load、one-step drift和fresh reset replay。
- state/sign：`base_control_frame` position/quaternion/Jacobian twist、orientation Log与small-angle/finite-difference一致；common wheel产生冻结的 longitudinal/pitch方向，differential wheel产生冻结的 yaw方向，positive `u_roll`产生world `+X` roll acceleration。
- sampled model：`2 ms × 5` 生成10 ms transition；保存步长 sweep、affine drift、authority/stabilizability、fit/holdout residual、closed-loop poles和torque/contact margin。离开constraint/contact流形的raw差分仅作diagnostic。
- nonlinear gate：至少覆盖 nominal、正负 pitch/roll rate、正负 rolling/yaw rate、X/Y force、roll/pitch/yaw moment和小型cross-coupled cases；每个正常case至少10 s。tuning与holdout分离，失败数据保留。

### Automated

- `./.venv/bin/python -m py_compile tools/experiments/solve_mujoco_3d_standing_equilibrium.py tools/experiments/validate_mujoco_3d_standing_contract.py tools/experiments/run_mujoco_3d_standing_prefreeze.py tools/experiments/run_mujoco_3d_standing_formal.py`：Phase 20工具语法通过。
- `./.venv/bin/python tools/experiments/solve_mujoco_3d_standing_equilibrium.py --output-dir data/experiments/<new-phase20-equilibrium-id>`：DG20-01证据写入新目录。
- `./.venv/bin/python tools/experiments/validate_mujoco_3d_standing_contract.py --output-dir data/experiments/<new-phase20-contract-id>`：DG20-02/03 state/input oracle通过。
- `./.venv/bin/python tools/experiments/run_mujoco_3d_standing_prefreeze.py --output-dir data/experiments/<new-phase20-prefreeze-id>`：DG20-04/05通过后才能开始Core实现。
- `cd ros_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-up-to wheel_leg_mujoco && colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco && colcon test-result --verbose`：构建、Core/ROS/Adapter/runner与旧mode兼容测试无失败。
- `./.venv/bin/python tools/experiments/run_mujoco_3d_standing_formal.py --output-dir data/experiments/<new-phase20-formal-id>`：冻结 formal matrix通过；同一输入在fresh replay目录满足manifest声明的exact/tolerance规则。
- `./.venv/bin/python tools/maintenance/test_mujoco_coordinate_contract.py`：canonical frame/order/sign、quaternion、三轴角速度和wheel direction回归。
- `./.venv/bin/python tools/experiments/run_mujoco_contact_floating_base.py --output-dir data/experiments/<new-phase20-phase18-regression-id>/raw`：Phase 18 full-3D contact/floating回归。
- `./.venv/bin/python tools/experiments/run_mujoco_planar_standing_formal.py --output-dir data/experiments/<new-phase20-phase19-regression-id>`：Phase 19 exact-planar mode与formal入口回归，不覆盖历史 evidence。
- `git diff --check`：文档与源码无空白错误。

### Formal Matrix / Evidence

- nominal equilibrium/reset/replay：双episode，验证首帧bilateral contact、position/heading anchor、旧command清除和exact/tolerance replay。
- 单轴：正负 pitch、roll、longitudinal velocity、yaw-rate，以及admissible时的lateral velocity；直接初态必须contact/equality-consistent，若lateral velocity与双轮no-slip流形不相容则使用冻结world-Y impulse，不伪造raw reset。
- 外扰：正负 world `X/Y` force、正负 roll/pitch/yaw moment；`Z` force仅作为height/contact outcome probe，不引入height controller。
- 结构扰动：左右同向与差分leg posture、common/differential wheel通道、model asymmetry/cross-coupling。
- 组合holdout：至少 lateral+roll、roll+yaw、pitch+yaw各一组正负或镜像case，且不得用于gain tuning。
- fault：left/right/either contact loss、invalid quaternion/non-finite、nonmonotonic/timing error、torque saturation；全部六路zero并锁存到reset。
- 每case检查 finite、`x/y/z`与三轴速度、orientation error/三轴角速度、joint posture/velocity、六路torque/limit、virtual-input mapping、双轮contact与normal-load positivity、rolling/lateral slip、penetration、closure、Adapter sign、5-step ZOH和final recovery。
- 动画或plot只用于观察，不作为PASS；REVIEW只读取真实raw/summary/manifest与命令结果。

## Acceptance Criteria

- [x] authoritative full-3D plant保持六自由度base、完整闭链和Phase 18 contact/solver不变量，没有planar/hidden约束或辅助外力。
- [x] zero-wheel-torque upright equilibrium和admissible reset/perturbation通过static/contact/closure/drift/replay gates。
- [x] `x_8`、orientation Log、heading anchor、common/differential wheel及roll-leg sign与canonical RobotState/TorqueCommand一致。
- [x] 三路虚拟输入对目标模态具有证据支持的authority/stabilizability；sampled model、gain、poles/residual与full nonlinear pre-freeze全部通过。
- [x] additive 3D Core mode、full-3D C++ loop、fault latch/reset/ZOH与default-zero、Phase 17、Phase 19兼容。
- [x] 冻结formal的全部正常/扰动case至少运行10 s；roll/pitch/heading恢复，`Y/Z`有界且速度衰减，bilateral contact、torque和finite门槛全部通过；plant-level slip/penetration/closure由Phase18回归继续覆盖。
- [x] left/right contact loss、invalid/nonmonotonic/timing/saturation均fail closed；fresh replay、non-overwrite、历史回归和reuse dry-run通过。
- [x] 所有模型/profile/source/binary/config/seed/case/threshold/input/output hash与supersedes关系写入manifest；旧evidence未覆盖。
- [x] REVIEW=`PASS` 且blocking findings为零后才创建RECORD并把ROADMAP改为complete；结论明确限制为current nominal simulation-only。

## Execution Notes

- 2026-08-26：Phase 20 PLAN建立。CBM确认公共RobotState/Adapter已具备完整3D sensing；主要缺口是roll/heading control authority与完整3D evidence，不是公共message。
- 2026-08-26：用户要求执行Phase 20；状态切换为`active`，从P20-T01与实现前DG20-01～DG20-05开始，任一证据门失败即停止增加控制复杂度。
- 2026-08-26：Graphify维护代理已对PLAN、ROADMAP和Phase索引执行最小增量更新；健康检查为2070 nodes、3640 links、29 hyperedges，dangling/missing/self-loop/collapsed均为0。
- 2026-08-26：equilibrium/contract/pre-freeze v5关闭DG20-01～05。v1整体轨迹回归与v4高增益LQR失败数据均保留；最终只冻结contact-mode内中心差分模型和通过独立holdout的v5 gain，准入Core实现。
- 2026-08-26：formal-v1在raw roll reset进入仿真前因单轮离地被拒绝；该非admissible reset不解释为controller FAIL。formal-v2用冻结的正负world-X moment覆盖roll方向，19个normal/perturbation与6个fault case全部PASS。
- 2026-08-26：formal-v2与fresh replay的25个CSV加summary共26个文件SHA-256 exact；worst normal值为`|x|=0.00176 m`、`|y-y0|=0.00153 m`、height error`0.000283 m`、pitch/roll/yaw=`0.00531/0.00476/0.00448 rad`、final linear/angular speed=`0.00153 m/s`/`0.0811 rad/s`，contact fraction 1.0，ZOH/sign/virtual mapping error均0。
- 2026-08-26：colcon共19 tests、coordinate contract、Phase18 plant regression、Phase19 formal regression和Phase20 equilibrium→contract→prefreeze fresh namespace dry-run全部PASS；formal non-overwrite返回2且目录清单不变。进入REVIEW。
- 2026-08-26：REVIEW初查发现formal-v2缺逐case wheel normal load/slip/penetration/closure列，故未判PASS。formal-v3在预先冻结的Phase18-derived门槛下补测并全部PASS：minimum normal load `30.17 N`、maximum penetration `0.000525 m`、rolling/lateral slip `0.00954/0.00157 m/s`、closure residual `0.000185 m`；26个fresh replay文件再次exact。CBM最终coverage显示改动源码metadata changed/new runner not tracked，已直接通读源码并用真实build/test/formal补足，不据旧图作完成结论。

## Blockers

None. 全部decision gates已关闭，REVIEW=`PASS`。
