# Phase 16: Controller ↔ MuJoCo 确定性闭环运行基线 — PLAN

Status: `complete`

## Goal

在不连接真机、不新增控制算法的前提下，把 current nominal MuJoCo、canonical RobotState/TorqueCommand、Controller Core 和 Adapter 组成固定步序、可复位、可重放、可记录的确定性闭环执行基线，使后续 Joint PD、重力补偿以及 identified plant 第二轮复现只替换 Controller/plant profile，不重写运行与证据链。

## Current State

- 已有：Phase 03 已冻结 ROS 无关 `ControllerCore`、canonical `RobotState`/`TorqueCommand` 和 ROS2 wrapper；当前 Core 对有效状态只输出六路零力矩。
- 已有：Phase 04 已交付 MuJoCo Adapter、ROS2 wall-paced node、双时钟/watchdog/reset 契约、one-hot 非零力矩映射测试，以及 fixed/floating zero-loop smoke。
- 已有：Phase 14/15 已验证 current nominal plant 的内部动力学、完整闭链、被动装配和 reduced Jacobian，并冻结 profile/hash/非覆盖复用要求。
- 缺少：Phase 04 的 ROS zero-loop 受 callback 与 wall timer 调度影响，没有一个以固定物理步数驱动、直接组合 Core 与 Adapter、可逐样本重放的正式 host runner。
- 缺少：当前没有统一的 control tick/zero-order hold 步序、Controller/plant/config manifest、逐步 state-command-ctrl 日志、跨 reset episode 等价性和跨 plant revision 比较入口。
- 边界：现有 one-hot Adapter 测试已经证明非零 `TorqueCommand → ctrl` 映射；本 Phase 不用临时 P/PD 或测试专用控制模式重复解决该问题。
- 路线：Phase 05 与所有真机工作继续保持 `blocked`；本 Phase 只建立 simulation-only 控制执行基线，不关闭任何 MuJoCo–real、接触或控制效果 gate。

## Scope

- Ground Phase 03/04 当前 Core、ROS wrapper、Adapter、MuJoCo node、时钟、reset、watchdog 和测试覆盖，明确本 Phase 相对 Phase 04 的新增证据，避免重做已 PASS 的映射工作。
- 建立 `phase16_contact_free.xml`：contact-disabled、保留命名 `floor` 和 current nominal 双腿闭链/`base_weld`，使 Adapter 对象契约保持不变，同时把轮地接触从执行基线中隔离。
- 冻结 physics tick、control tick、状态采样、Controller step、命令接收、zero-order hold、MuJoCo step 和日志写入的唯一顺序。
- 实现 ROS 无关的 C++ deterministic loop executable，直接链接现有 `wheel_leg_core` 与 `wheel_leg_mujoco_adapter`；不复制 state/command 映射，不让 Core 依赖 MuJoCo。
- 建立薄 Python experiment wrapper，负责 profile/config 解析、进程执行、SHA-256 manifest、输出目录非覆盖、结果汇总和跨 run 比较；物理循环与 Controller 调用不在 Python 中重写。
- 记录每个 control tick 的 source/receipt time、RobotState、Core status/`dt`、TorqueCommand、native ctrl、physics-step range 和 reset epoch；失败样本不得静默丢弃。
- 验证固定步序、零阶保持、双时钟分离、command freshness、失效归零、显式 reset、同输入重复执行和 episode replay。
- 回归 Phase 02/04/14/15 相关验证，并形成可由未来 Controller 版本、SolidWorks revision 和 identified profile 原样复用的 manifest/log schema。

## Out of Scope

- Joint P/PD、重力补偿、轨迹跟踪、轮速控制、站立、WBC、NMPC 或任何控制效果/稳定性结论。
- 为制造非零输出而向 production Core 添加 validation-only 控制模式、回调注入、策略接口或临时增益。
- 修改 canonical joint order、FLU frame、符号、RobotState/TorqueCommand schema、Adapter 映射或 ROS topic 名称。
- 轮地接触、摩擦、滑移、接触力、floating-base 落地或接触求解器保真度验证。
- 修改或辨识 mass、COM、inertia、friction、armature、actuator scale/deadzone/delay 或真机参数。
- 任何真机上电、STM32/树莓派联调、传感器采集、Load Cell、执行器辨识或 Hardware Adapter 工作。
- 实时性认证；ROS wall-paced smoke 只能支持 transport compatibility，不能替代固定步数正式证据。
- 覆盖 Phase 14/15 evidence、current nominal model 或任一历史 run。

## Frozen Decisions

- Phase 16 的正式验收对象是 current nominal plant；模型 revision/hash、Controller source hash、config hash 和 runner hash 都必须进入 manifest。未来 revision/identified profile 产生新 run，不继承本 Phase 数值 PASS。
- 当前 `ControllerCore` 的六路零力矩行为保持不变。本 Phase 验收执行/时间/生命周期/日志正确性，不用闭环轨迹证明尚不存在的控制算法。
- 非零力矩链路继续由 Phase 04 one-hot Adapter regression 负责；真正由 Controller 产生的非零反馈命令留给 Joint PD/重力补偿 Phase，并复用本 Phase runner。
- 正式 fixture 使用 current nominal `wheel_leg.xml`、全部双腿闭链 equality 和 `base_weld`；场景保留命名 `floor` 供 Adapter 解析，但全局禁用 contact。不得用 floating-base 或地面碰撞掩盖执行链问题。
- physics timestep 固定为 `0.002 s`；control period 固定为 `0.010 s`，即每个命令保持 5 个 physics steps。两者必须是整数比，runner 对非整数配置直接失败。
- 每个 control tick `t_k` 的顺序固定为：从当前 `mjData` 提取 `RobotState(t_k)` → `ControllerCore::step` → 接收同源时间的 `TorqueCommand(t_k)` → 写入/保持 ctrl → 推进 5 个 physics steps → 记录下一 tick。命令作用于 `[t_k,t_{k+1})`，不允许隐式提前或多一拍延迟。
- `RobotState.sample_time_ns` 继续来自 `mjData.time`；deterministic runner 使用独立、单调的 logical receipt clock 测试 watchdog。两个时钟允许同速推进，但字段、比较和日志必须分开，禁止互相相减冒充同一时钟域。
- reset epoch 顺序固定为 MuJoCo data/Adapter reset 后 Controller reset；旧 epoch 命令不得跨 reset 生效。相同初值/config 的 reset replay 必须与 fresh episode 逐样本一致。
- 正式重复性要求同一二进制、模型、config 和环境下两次 fresh run 的规范化 CSV/summary SHA-256 相同；环境或二进制变化时只做数值字段比较，不声明 bitwise determinism。
- runner/config/schema 必须 profile-driven；切换 plant 或 Controller 版本不得编辑循环算法。非空输出目录必须拒绝覆盖。
- C++ executable 负责真实闭环步进；Python wrapper 只做编排、hash、schema 检查与报告，避免形成第二套动力学或 Controller 实现。
- 不为 runner 新增第三方依赖；实现复用现有 C++17、MuJoCo、ROS package 和 Python 标准库能力。

## Open Questions / Decision Gates

- **DG01 / CLOSED / CODEX_DECISION — Phase 04 重叠边界：** Phase 04 的 ROS zero-loop、Adapter mapping 和 fail-safe 作为回归输入；Phase 16 新增的是确定性 in-process 调度、逐步日志、replay 和版本化实验入口，不重新认领 Phase 04 PASS。
- **DG02 / CLOSED / CODEX_DECISION — 非零反馈：** 本 Phase 不添加临时反馈律；zero Core 证明执行闭环，Phase 04 one-hot test 证明非零映射，首个 production 非零反馈由后续 Joint PD Phase 负责。
- **DG03 / CLOSED / CODEX_DECISION — 正式调度：** `2 ms` physics、`10 ms` control、5-step ZOH，状态 `t_k` 产生并标记命令 `t_k`，命令作用于下一 control interval。
- **DG04 / CLOSED / EVIDENCE — replay determinism：** 正式运行的 fresh/fresh CSV SHA-256 相同，跨进程与 reset/fresh 最大数值差均为 `0`；200 行 nominal tick 的 epoch、时间和步数完整。
- **DG05 / CLOSED / EVIDENCE — lifecycle/fail-safe：** 正式 fault log 证明 duplicate/non-monotonic state、future/stale/timeout 和 reset-old command 被拒绝或写零，timeout 后与新 epoch 合法命令均恢复。
- **DG06 / CLOSED / CODEX_DECISION — evidence authority：** 固定步数 C++ runner 是确定性结论的正式来源；ROS2 launch 只做 topic/schema/reset compatibility smoke，不使用 wall-clock jitter 判定数值重放 PASS。

## Interfaces and Compatibility

- 输入：显式 model/scene、plant profile、Controller build/source、physics/control timing、episode length、initial state/seed、fault schedule、threshold config 和新 output run ID。
- Controller 边界：Phase 03 `ControllerCore::configure/reset/step(RobotState)`；不得绕过 Core validation 或直接修改其 accepted-sample history。
- Plant 边界：Phase 04 `Adapter::extractState/acceptCommand/writeControls/reset` 与 MuJoCo `mj_step`；不得复制 joint offset/sign/order/contact 逻辑。
- 输出：run manifest、每 control tick CSV、episode/reset/fault summary JSON、determinism comparison 和自动 evidence。native/canonical、source/receipt、physics/control tick 字段必须显式区分。
- 必须保持：Phase 02 坐标契约、Phase 03 package 依赖方向、Phase 04 Adapter/watchdog/reset 行为、Phase 14/15 nominal evidence 和 Phase 05 blocked 状态。
- 允许改变：新增 Phase 16 scene/config、C++ deterministic executable、Python experiment wrapper、测试、方法文档和追加式 evidence；若必须修改现有 Core/Adapter，只允许修复由真实失败暴露的契约 bug，并要求相应旧 Phase 回归。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| T01 | Ground 现有 Core↔ROS↔Adapter↔MuJoCo 执行链并冻结增量边界 | Phase 03/04 RECORD、Core/wrapper/node/Adapter、Phase 14/15 profile | grounding evidence、调用/时钟/reset/测试覆盖图、DG01/DG02 关闭记录 | 每个职责映射到当前符号和测试；Phase 04 已 PASS 内容不被重复包装成新结论 | done |
| T02 | 冻结 Phase 16 nominal scene/profile/config 与调度契约 | `wheel_leg.xml`、Phase 04 timing、Phase 14/15 manifest 规则 | `phase16_contact_free.xml`、`phase16_nominal.json`、对象/timestep/equality invariants | MuJoCo 3.7.0 加载；`floor`/6 joints/6 actuators/base weld/闭链名称存在；contact disabled；2 ms/10 ms 整数比 | done |
| T03 | 实现 ROS 无关 deterministic loop executable | T01/T02、`wheel_leg_core`、`wheel_leg_mujoco_adapter` | 单进程固定步数 loop、显式 epoch/reset/fault入口、CMake target | 单元/集成测试证明固定调用顺序、5-step ZOH、source-command timestamp identity、Core/Adapter 无反向依赖 | done |
| T04 | 建立逐 tick 日志、manifest 和非覆盖 wrapper | T02/T03、两轮复现契约 | `run_mujoco_controller_loop.py`、CSV/JSON schema、SHA-256 manifest、run comparison | 输入/二进制/Controller/scene/config hash 完整；非空目录失败；解析失败不产生 PASS summary | done |
| T05 | 验证双时钟、命令生命周期与 fail-safe，关闭 DG05 | T03/T04、冻结 fault schedule | duplicate/stale/future/timeout/dropout/reset-old 场景及完整日志 | 非法路径不推进 Core accepted history或不进入 ctrl；缺失/过期命令全零；合法新 epoch 可恢复 | done |
| T06 | 验证 fresh replay、reset replay 和确定性，关闭 DG04 | nominal config、固定 episode、相同 seed/initial state | 两次 fresh run、同进程 reset episode、字段级 diff 与 hash comparison | 同环境规范化输出 hash 相同；所有样本有限；时间/步数/epoch 无丢失、重复或漂移 | done |
| T07 | 执行 ROS compatibility 与历史回归 | T01–T06、现有 launch/tests、Phase 02/14/15 runner | colcon/test、ROS topic/reset smoke、Phase 02/14/15 新目录回归记录 | ROS schema/topic/reset 兼容；coordinate、Adapter、内部动力学、闭链运动学无回退；旧 evidence 未修改 | done |
| T08 | 固化复用文档、正式 evidence 并准备 REVIEW | T01–T07 | `docs/experiments/mujoco_controller_loop_validation.md`、README 入口、automated evidence、Execution Notes、REVIEW 输入 | 新 run 可按文档复现；DG01–DG06 关闭；不声明 PD、接触、实时性、真机或模型准确性 PASS | done |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Automated

- `./.venv/bin/python tools/maintenance/test_mujoco_coordinate_contract.py`：Phase 02/04 坐标、joint sign、COM frame 和 nominal model contract 继续 PASS。
- `cd ros_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-up-to wheel_leg_mujoco`：新增 executable 与现有 Core/Adapter 在 Jazzy、MuJoCo 3.7.0 下构建通过。
- `cd ros_ws && source /opt/ros/jazzy/setup.bash && colcon test --packages-select wheel_leg_core wheel_leg_ros wheel_leg_mujoco && colcon test-result --verbose`：调度、ZOH、reset、fault 和现有 mapping/pub-sub tests 无失败。
- `./.venv/bin/python tools/experiments/run_mujoco_controller_loop.py --profile nominal --output-dir data/experiments/<new-run-id>/raw`：执行正式 fixed-base/contact-disabled 两 episode run，输出 manifest、tick CSV、summary 与 comparison；全部 gate PASS。
- 对同一 config 使用两个新 output run ID 重跑：规范化 CSV/summary hash 相同；字段级 diff 的最坏值为零。不得复用非空目录。
- 使用冻结 fault profile 运行 duplicate state、stale/future/timeout/dropout/reset-old command 场景：拒绝状态、零 ctrl、epoch 恢复和日志完整性与预期一致。
- `./.venv/bin/python tools/experiments/run_mujoco_internal_dynamics.py --output-dir data/experiments/<new-phase14-regression-id>/raw`：Phase 14 九项内部动力学基线继续 PASS。
- `./.venv/bin/python tools/experiments/run_mujoco_closed_chain_kinematics.py --output-dir data/experiments/<new-phase15-regression-id>/raw`：Phase 15 210 样本闭链/Jacobian 基线继续 PASS。
- `ros2 launch wheel_leg_mujoco zero_loop.launch.py ...` 加自动 topic/reset probe：只验证 transport/schema/reset compatibility、有限值和零输出，不把 wall-clock jitter 计入 deterministic verdict。

### Manual / Evidence

- Codex 审查正式 tick 顺序、physics/control 计数、source/receipt 时钟列、每个 reset epoch、全部拒绝样本和 worst-case diff；不能只看 `overall_pass`。
- 正式 run manifest 至少记录 model/profile/scene/config/Controller/runner hash、MuJoCo/ROS/compiler 版本、physics/control period、初值/seed、fault schedule、阈值和输出 schema version。
- REVIEW 核对 current Core 仍为零输出、未新增 validation-only 控制模式；六路 command/native ctrl 的零值与 timestamp/hold 行为均由真实日志支持。
- REVIEW 核对 Phase 14/15 正式 evidence 未覆盖，两个 fresh run 和 reset episode 可同时保留、解析和比较。
- 若 SolidWorks revision 在 Phase 执行期间到达，只允许新增独立 smoke profile/run；current nominal 仍是本 Phase 唯一必需验收对象，除非另行记录 scope decision。

## Acceptance Criteria

- [x] T01–T08 完成，DG01–DG06 全部关闭且无未记录偏差。
- [x] current nominal scene 在 fixed-base、contact-disabled 条件下，以 2 ms physics、10 ms control 和 5-step ZOH 执行；调用顺序和时间语义与 Frozen Decisions 一致。
- [x] 每个 accepted RobotState 产生同 source timestamp 的 TorqueCommand；当前 Core 六路 command 与 native ctrl 全零，所有状态/日志字段有限。
- [x] fresh/fresh 与 reset/fresh replay 在同环境下规范化输出 hash 一致；步数、control tick、epoch、source time 和 receipt time 无丢失、重复或漂移。
- [x] duplicate/non-monotonic state、future/stale/timeout/dropout/reset-old command 均按契约拒绝或失效归零，且下一合法 epoch 可恢复。
- [x] runner/config/log schema 可在不改循环算法的情况下切换未来 Controller build、SolidWorks revision 和 identified plant profile；所有输入和版本有 hash。
- [x] 非空 output directory 被拒绝，新的 run/evidence 追加保存，Phase 14/15 和 current nominal baseline 未覆盖。
- [x] ROS compatibility、coordinate contract、Adapter、Phase 14 和 Phase 15 回归继续 PASS。
- [x] 方法文档、README、ROADMAP、实现和真实 evidence 一致；不声明控制效果、接触保真度、实时性或真机一致性。

## Execution Notes

- 2026-08-25：根据用户要求只制定 Phase 16，不开始实现。Phase 目标限定为 Controller↔MuJoCo 确定性执行与证据基线，状态保持 `planned`。
- 2026-08-25：Grounding 确认 Phase 04 已有 ROS wall-paced zero-loop、非零 one-hot Adapter mapping、watchdog/reset 和 fixed/floating smoke；Phase 16 不重复这些结论，只补固定步数 in-process runner、逐 tick 日志和 replay/non-overwrite 入口。
- 2026-08-25：采用 ponytail 最小边界：保留 current Core 零输出，不添加临时 P/PD、策略接口或第三方运行依赖；首个 production 非零反馈留给后续 Joint PD Phase。
- 2026-08-25：Graphify 仅查询了现有本地图以核对 Phase 14/15 与总体两轮路线，没有执行 extract/update 或修改 `graphify-out/`。
- 2026-08-25：用户授权执行 Phase 16；状态切换为 `active`。CBM generation `2026-08-25T06:16:31Z` 确认 Core/Adapter 当前符号与调用边界，`adapter.hpp:28` 的 partial coverage 已直接读取补足；Graphify 仍仅查询现有图。
- 2026-08-25：新增 contact-disabled nominal scene、versioned config、ROS 无关 C++ fixed-step runner 与标准库 Python wrapper。正式 200-tick nominal、fault schedule、fresh/reset replay 和全部输入/输出 hash gate 通过。
- 2026-08-25：ROS regression 初次受到用户默认 Domain 中正在运行节点的消息干扰；未停止用户进程，改为独立测试 Domain 并收紧单样本等待逻辑后，18 tests 全部 PASS。独立 Domain launch topic/reset smoke、坐标契约和 Phase 14/15 新目录回归均 PASS。
- 2026-08-25：T01–T08 与 DG01–DG06 全部关闭，停止实现扩张并进入 `review`。
- 2026-08-25：REVIEW 检查发现并修复 profile 名限制与 ROS test Domain 污染；新目录正式证据及全部回归通过，Verdict `PASS`，创建 RECORD，状态切换为 `complete`。

## Blockers

None. 本 Phase 不依赖真机、STM32、Load Cell、Hardware Adapter、identified 参数、轮地接触或新的 Controller 算法。
