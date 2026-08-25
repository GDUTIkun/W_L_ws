# Phase 04: MuJoCo 基础模型与 Adapter — PLAN

Status: `complete`

## Goal

在不引入控制算法或已标定物理参数结论的前提下，建立复用 Phase 03 公共类型的 `wheel_leg_mujoco` Adapter 和可重复 headless 运行入口，使 `MuJoCo → RobotState → 当前零输出 Controller Core → TorqueCommand → MuJoCo` 的六关节闭环通路以明确的坐标、时间、复位、接触和失效安全语义运行并通过自动验证。

## Current State

- 已有：Phase 02 已冻结 canonical `{N}` 为 FLU、torso `base_control_frame` 为控制基座 frame、active quaternion 为 `[w,x,y,z]`、六关节顺序，以及 `q_C=-q_M+b_joint`、`dq_C=-dq_M`、`tau_M=-tau_C` 的符号关系。
- 已有：Phase 03 已交付 ROS 无关的 `wheel_leg_core::RobotState` / `TorqueCommand`、安全零输出 Controller Core、聚合 ROS messages 和 `wheel_leg_ros` wrapper，并通过 ROS2 Jazzy build/test。
- 已有：[`wheel_leg.xml`](../../../../simulation/mujoco/model/wheel_leg.xml) 包含完整双腿 CAD 多刚体、base free joint 和左右闭链 connect constraints；[`scence.xml`](../../../../simulation/mujoco/model/scence.xml) 使用 FLU gravity、`0.002 s` timestep 和平面地面。
- 已有：[`test_mujoco_coordinate_contract.py`](../../../../tools/maintenance/test_mujoco_coordinate_contract.py) 已回归 COM site、world/joint axes、左右微扰、wheel rolling 和 quaternion 语义；[`audit_mujoco_runtime.py`](../../../../tools/maintenance/audit_mujoco_runtime.py) 可导出编译模型清单和运行时 probe。
- 缺少：当前 MJCF 没有 actuator，base free joint 被 world weld 约束，wheel collision geoms 没有稳定名字，且尚无 MuJoCo C++ package、状态提取、力矩写入、接触聚合、复位协议或 ROS2 运行闭环。
- 缺少：六个 `b_joint` 仍未通过 matching pose 与第二姿态回归冻结；raw MuJoCo qpos 不得直接进入 Controller state。
- 已发现契约冲突：Phase 03 将 `sample_time_ns` 同时解释为 Controller host steady clock，并由 Core 用 `now_ns` 判断 stale/future；确定性仿真应以 `mjData.time` 表示源采样时间，不能与 host steady clock 直接相减。Phase 04 必须先完成受控契约修订。
- 当前工具链：ROS2 Jazzy/`colcon` 可用；本机存在 `/opt/mujoco-3.7.0` C++ headers/library，但当前 Python 环境未安装 `mujoco`，而 `simulation/mujoco/environment.yml` 声明 `mujoco==3.12.0`。版本未统一前不得生成跨语言 PASS 证据。

## Scope

- Ground 编译后 MJCF 的 body/joint/site/geom/sensor/equality/actuator IDs、qpos/qvel/ctrl addresses、timestep、约束拓扑和现有运行工具链，形成可审计清单。
- 将公共时间契约修订为“源单调采样时间 + 本机 receipt steady clock”两个域；Core 只使用源时间做顺序和 `dt`，transport/Adapter 使用 receipt clock 做 watchdog；仿真复位必须显式同步 reset，禁止自动接受时间回退。
- 用数值 matching pose 冻结六个 `b_joint`，并以第二个非零姿态的 canonical FLU 几何回归证明 offset 不是单点凑合。
- 为六个驱动关节建立无隐藏符号/比例的直接力矩 actuator，并实现 canonical torque 到 MuJoCo ctrl 的显式顺序、符号和安全门控。
- 实现 ROS 无关、可单测的 MuJoCo Adapter 映射层：`MjData → RobotState`、`TorqueCommand → ctrl`、对象名解析、contact 聚合、reset 和错误返回。
- 实现最小 ROS2/headless runner；物理步进、状态发布和命令消费使用明确调度，默认无有效/启用命令时始终写入六路零力矩。
- 建立 fixed-base mapping/闭环 smoke 与 floating-base state/reset sanity 两类场景；floating-base 不承担站立或控制效果验收。
- 建立模型、映射、时间、reset、接触、力矩、determinism、ROS pub/sub 和有限值自动测试，保存真实命令与输出证据。
- 更新 MuJoCo、ROS workspace 和 package 入口文档，明确已验证能力与后续参数/控制层边界。

## Out of Scope

- Planner、NMPC、WBC、Joint PD、重力补偿、姿态稳定、站立、轮子速度控制或任何其他控制算法。
- Phase 05 的真实执行器 torque scale/bias、deadzone、摩擦、反射惯量和驱动限制标定；Phase 04 actuator 只证明接口和符号映射。
- Phase 06 的真实 encoder/IMU 安装、传感器滤波、延迟、生产通信和 Hardware Adapter 验证。
- Phase 07/08 的完整 FK/Jacobian、质量/COM/inertia、正逆动力学、接触参数和 MuJoCo–真机一致性结论；matching-pose 回归只服务于 joint offset。
- 地形、轮地摩擦调参、碰撞几何保真度、viewer/UI、实时性认证或长时间控制性能测试。
- 修改第三方 mesh/CAD 资产，或把 nominal/imported 参数描述为已校准真机参数。

## Frozen Decisions

- Phase 02 的 FLU、SI、`base_control_frame`、active `[w,x,y,z]`、六关节顺序和 joint sign/torque power-conjugate 映射保持不变；左右两侧不增加镜像负号。
- Phase 03 的 `RobotState` / `TorqueCommand` 字段与 ROS message schema保持不变；Phase 04 只允许受控修订时间/生命周期解释和相关 Core/wrapper API，不新增 simulator-only 字段。
- `sample_time_ns` 表示源系统的单调采样时间：MuJoCo Adapter 由 `mjData.time` 确定性换算为 ns；host steady clock 仅用于 receipt age/watchdog，两个时钟域不得直接相减。
- 仿真 reset 必须同时清空 Adapter command/history 和 Controller accepted-sample history；时间回退在没有显式 reset 的情况下仍是错误。
- MuJoCo Adapter 依赖 `wheel_leg_core`/`wheel_leg_msgs`，Core 不得依赖 MuJoCo、ROS 或 runner。映射逻辑必须能脱离 ROS node 单元测试。
- MuJoCo object 地址只允许在加载时按冻结名称解析并缓存；不得依赖导入顺序或硬编码全局 qpos/qvel/geom 数字索引。
- `RobotState` base pose/twist 取 torso `base_control_frame`：pose 表达在 `{N}`；linear/angular velocity 都是该 site 原点相对 `{N}` 的速度并表达在 `{N}`。twist 必须用 site Jacobian 与有限差分/刚体速度交叉验证，不读取 raw `base_frame` sensor 代替。
- 六个 actuator 使用 unit gear 的 MuJoCo native direct torque 语义；`tau_M=-tau_C` 在 Adapter 中显式实现，不隐藏在 gear、左右特例或 message 排列中。
- Adapter 默认 torque path 为 disabled/zero；只有通过配置安全门、有限值、source-time 和 receipt-time 检查的命令才能进入 ctrl。缺失、超时、乱序、future、reset 前或非法命令都写六路零。
- contact 是 simulator-only ground-truth aggregation：按命名的左右 wheel collision geom 与环境碰撞聚合为 `[left,right]`；已完成 contact evaluation 且无匹配 contact 为 `no-contact`，状态不可用时为 `unknown`。不由 force threshold 推导，接触力保真度不在本 Phase 声明。
- `0.002 s` 物理 timestep 保持为本 Phase baseline；fixed-base 用于 mapping/闭环 smoke，floating-base 只用于 pose/twist/contact/reset/有限值 sanity。两类场景必须显式选择，不能靠测试中不可见的临时模型变异。
- 不复制或重写现有模型；优先通过小型 scene/config、命名和 Adapter 层扩展保持 Phase 02 regression 可运行。

## Open Questions / Decision Gates

- **DG01 / CLOSED / CODEX_DECISION — MuJoCo 版本与构建链：** C++、Python 和 environment pin 统一为 MuJoCo 3.7.0；两端加载同一 scene。
- **DG02 / CLOSED / CODEX_DECISION — 时间与 reset API：** Core 只处理 source order/`dt`；Adapter 用 receipt steady clock watchdog；reset 顺序冻结为 simulation 后 controller。
- **DG03 / CLOSED / EVIDENCE — joint zero offsets：** 六路 `b_joint` 已从编译模型几何冻结，并通过第二非零姿态回归。
- **DG04 / CLOSED / EVIDENCE — base twist 提取：** `mj_jacSite*qvel` 的 COM-site/world twist 已与有限差分和已知刚体运动交叉验证。
- **DG05 / CLOSED / CODEX_DECISION — collision/contact set：** 仅命名左右 wheel collision geom 与 `floor` 的配对进入 contact 聚合。
- **DG06 / CLOSED / EVIDENCE — runner 调度与 watchdog 参数：** physics 2 ms、state decimation 5、source lag 50 ms、receipt timeout 100 ms；ROS 实测 state 约 99.99 Hz。
- 当前没有 `USER_CONFIRMATION` gate；上述事项均可由现有契约、源码和真实运行证据关闭。若 matching-pose 所需的 Simulink reference 无法从仓库稳定生成，T03 转 `blocked` 后再请求用户提供/确认 reference pose 数据。

## Interfaces and Compatibility

- 输入模型：明确选择的 fixed-base 或 floating-base MJCF scene；加载后按名称解析 `base_control_frame`、六个驱动 joint/actuator、左右 wheel collision geoms 和 equality constraints。
- 输入命令：Phase 03 `wheel_leg_core::TorqueCommand` / `wheel_leg_msgs::msg::TorqueCommand`；六关节 canonical 顺序、单位 `N·m`，并携带产生它的 source sample time。
- 输出状态：Phase 03 `wheel_leg_core::RobotState` / `wheel_leg_msgs::msg::RobotState`；base COM pose/twist、六关节 canonical q/dq、左右 tri-state contact 和 source simulation time。
- 调度：MuJoCo physics 由 runner 以固定 `0.002 s` step 推进；状态发布按 simulation-time accumulator/整数 decimation 触发；ROS callback 只交换状态/命令，不改变积分步长。
- 复位：runner 发起显式 reset sequence，清理 `mjData`、Adapter command/history 和 Controller history，再从 source time 0 开始；reset 前命令不可复用。
- 必须保持：Phase 02 coordinate regression、Phase 03 schema/package 依赖方向、Phase 05 实验 package 行为和原始 mesh 资产。
- 允许改变：新增 `wheel_leg_mujoco` package、scene/config/test/evidence；为 MuJoCo object 增加稳定名字和六个 unit-gear actuator；对 Phase 03 时间契约/Core/wrapper 做 DG02 所需的最小兼容修订及测试更新。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| T01 | Ground 当前 MJCF、运行工具链和版本，关闭 DG01 | Phase 02/03 RECORD、MJCF、Python audit、ROS2 与 `/opt` MuJoCo | `evidence/mujoco_grounding.md`、可机读 compiled manifest、统一版本/环境说明 | 同一版本下 C++ 与 Python 均能编译/加载同一 scene；记录 `nq/nv/nu`、IDs/addresses、constraints、timestep 和工具版本 | done |
| T02 | 修订双时钟与 reset 生命周期，关闭 DG02 | Phase 03 contract/Core/wrapper、T01 toolchain | 更新的接口契约、Core/wrapper API 与 reset handshake、回归测试；不改变消息 schema | source time 的 order/`dt`、receipt timeout、future/late、显式 reset/非法 rollback 均有独立测试；Core 不读取 host clock | done |
| T03 | 冻结六关节 offset，关闭 DG03 | joint mapping evidence、matching pose、MuJoCo/Simulink reference | `evidence/joint_offset_calibration.md`、六路配置/fixture、两个姿态期望值 | `b_joint=q_C+q_M` 逐路可追溯；第二姿态 hip/knee/wheel-center FLU regression 在预先记录容差内 | done |
| T04 | 建立显式场景、稳定对象名、actuator 与 contact set，关闭 DG05 | T01/T03、现有 MJCF | fixed/floating scene/config、六个 unit-gear actuator、命名 wheel collision geoms、加载时 invariant checks | Phase 02 coordinate test 不回退；模型 `nu=6`；actuator/order/gear、scene mode、collision names 和 equality 状态断言通过 | done |
| T05 | 实现可单测的 MuJoCo Adapter 映射层，关闭 DG04 | T02–T04、MuJoCo C API、Phase 03 types | `wheel_leg_mujoco` library：model binding、state extraction、contact aggregation、command mapping、zero fail-safe、reset | pose/quaternion、site twist、q/dq offset/sign/order、contact、`tau_M=-tau_C`、NaN/Inf/timeout/乱序/reset tests 全通过 | done |
| T06 | 实现 headless/ROS2 runner 并关闭 DG06 | T02/T05、ROS2 Jazzy | node/runner、参数/config、simulation-time scheduler、wall pacing、reset entrypoint 和 launch | 固定步数确定性测试；callback 不改变 physics `dt`；超时/停发/reset 后 ctrl 立即归零；记录实测速率 | done |
| T07 | 完成跨层自动验证与零输出闭环 | T01–T06、当前 Controller Core | fixed-base ROS zero-loop、floating-base sanity、自动证据报告和必要 JSON/CSV artifacts | `MuJoCo → RobotState → Core → TorqueCommand → ctrl` 顺序/时间一致；bounded run 无 NaN/Inf；同 seed/reset 重复结果一致 | done |
| T08 | 更新入口文档并准备 Phase REVIEW | 全部实现与真实证据 | package/`ros_ws`/MuJoCo README、Execution Notes、evidence index 和 REVIEW 输入 | 新环境可从文档复现 build/test/run；所有 DG 关闭；不声明控制、标定或真机 PASS | done |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Automated

- `python3 tools/maintenance/test_mujoco_coordinate_contract.py`：在 T01 冻结的 Python 环境中继续 PASS，证明 Phase 02 coordinate contract 未回退。
- `python3 tools/maintenance/audit_mujoco_runtime.py --scene <phase-04-scene> --output <phase-04-manifest>`：成功编译模型并记录版本、addresses、`nu=6`、gravity、timestep、constraints、sites/sensors/actuators；脚本须扩展后再运行，不能复用旧 manifest 冒充新证据。
- `colcon build --symlink-install --packages-up-to wheel_leg_mujoco`：ROS2 Jazzy 下构建 Core/messages/wrapper/Adapter，无 ROS/MuJoCo 反向依赖。
- `colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco && colcon test-result --verbose`：双时钟/reset、offset、state、twist、contact、torque、fail-safe、determinism 和 ROS integration tests 全部通过。
- 新增模型 invariant test：按名字解析六个 canonical joints/actuators，检查 qpos/dof/ctrl 地址唯一、unit gear、fixed/floating equality mode、命名 collision geoms 和 `0.002 s` timestep。
- 新增 offset test：matching pose 计算六个 `b_joint`，第二非零 pose 对比 canonical hip/knee/wheel-center；容差在 fixture 中先冻结再验证。
- 新增 twist test：对纯平移、纯转动和组合运动，用 `mj_jacSite*qvel` 与有限差分/site pose 结果交叉验证，覆盖 world expression 与 COM-origin lever-arm 项。
- 新增 torque test：六路 one-hot canonical torque 逐个验证唯一对应 native ctrl 且符号为负；disabled、missing、NaN/Inf、source lag、receipt timeout、乱序和 reset 后均为全零。
- 新增 contact test：左右 wheel–environment contact、no-contact、unavailable 三态分别覆盖；交换左右或把自碰撞误计入时测试必须失败。
- headless fixed-base smoke：运行冻结时长/步数的 ROS zero-loop，保存 sample/source time、q/dq、command/ctrl 摘要，要求六路输出为零、时间严格推进且无 NaN/Inf。
- headless floating-base sanity：在显式 reset 后运行有界 free-fall/state extraction，不要求站立，只要求 pose/twist/contact 语义、重力方向、reset 重放和有限值一致。

### Manual / Evidence

- T01 保存 OS、compiler、ROS distro、MuJoCo C++/Python 精确版本、library/header discovery 和全部复现命令；版本或加载路径不同则验证无效。
- T03 的 matching pose 必须附 Simulink/Controller 与 MuJoCo 数值来源、六个 joint 值和 canonical FLU frame 点位；若必须人工选 pose，只让用户确认输入数据，不把肉眼“看起来一致”作为 PASS。
- T06 记录 headless 与 ROS wall-paced 的实际 physics/state/command rate、最大 source lag 和 receipt age；默认 timeout 必须由测量加明确裕量得到。
- T07 保存固定步数、初值/seed、scene hash、配置 hash 和摘要；fixed/floating 两种模式分别解释，不能用 fixed-base 结果支持 floating-base 控制结论。
- REVIEW 逐项核对 clock domain、reset、joint offset、torque sign、contact set 和 fail-safe；任一 DG 未关闭或只有静态代码证据时 Verdict 必须为 `REWORK`。

## Acceptance Criteria

- [x] T01–T08 全部完成，DG01–DG06 全部由文档、代码和真实运行证据关闭。
- [x] C++ 与 Python 使用同一精确 MuJoCo 版本；模型/scene 可重复加载，Phase 02 coordinate regression 保持 PASS。
- [x] 六个 `b_joint` 有 matching pose 与第二非零姿态几何回归；raw qpos 未直接发布为 canonical q。
- [x] `RobotState` 的 source time、COM pose/twist、六关节 q/dq 和左右 contact 均符合冻结语义；clock reset 不会被误判为正常单调样本。
- [x] 六个 unit-gear actuator 的 canonical order 和 `tau_M=-tau_C` 由 one-hot 自动测试覆盖；默认、失效、超时、乱序和 reset 后 ctrl 均严格为零。
- [x] fixed-base headless ROS zero-loop 与 floating-base bounded sanity 均可从文档复现，运行无 NaN/Inf，reset 后确定性重放一致。
- [x] Core 仍不依赖 MuJoCo/ROS/transport；Adapter mapping 能脱离 ROS 单测，ROS callback 不控制 physics `dt`。
- [x] 自动证据记录真实命令、环境、scene/config hash 和结果；README/ROADMAP 不宣称控制效果、参数标定或真机一致性 PASS。

## Execution Notes

- 2026-08-25：创建 Phase 04 PLAN，状态保持 `planned`；本次只冻结范围、架构约束、decision gates、任务和验证设计，未开始实现。
- 2026-08-25：CBM 对 `simulation/mujoco` 和 Phase 03 Core 目标路径无已记录覆盖缺口；`tools/maintenance` 按项目规则未索引，已直接读取 coordinate/audit 脚本补足 grounding。
- 2026-08-25：Graphify 历史图确认 Phase 02 joint mapping/coordinate evidence、Phase 03 公共边界和分层放行路线均指向 Phase 04；当前源码与历史设计未发现需要用户选择的冲突。
- 2026-08-25：发现 Phase 03 host-steady 时间定义与确定性 `mjData.time` 接入冲突，登记为 DG02/T02；修订限制在时间职责和 reset 生命周期，不改变公共消息字段。
- 2026-08-25：检测到 `/opt/mujoco-3.7.0` C++ library/header，但 active Python 缺少 `mujoco`，且 YAML 声明 3.12.0；登记为 DG01，版本统一前不运行或声称 Phase 04 验证 PASS。
- 2026-08-25：状态依次进入 `active` 与 `review`；完成 T01–T08、关闭 DG01–DG06，并以真实 C++/Python/ROS fixed/floating 验证进入审查。
- 2026-08-25：REVIEW 无 blocking finding，Verdict `PASS`；创建 RECORD 并将 Phase 状态更新为 `complete`。

## Blockers

None. 执行必须从 T01 开始；T03 依赖可复现的 Simulink matching-pose 数据，T04–T06 不得在相应 decision gate 未关闭时越过实施。
