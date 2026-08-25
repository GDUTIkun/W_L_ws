# Phase 02: 坐标系、单位、关节顺序与接口语义 — PLAN

Status: `active`

## Goal

以已验证的 Simulink 控制语义为基准，冻结 Simulink、MuJoCo、Controller Core 和真机共同使用的坐标、方向、单位、关节顺序与状态/命令表达契约；允许 MuJoCo 内部使用方便的相对 body/site frame，但任何跨边界变换都必须显式记录并通过方向性测试。

## Current State

- 已有：Phase 01 已迁入 MATLAB R2024b 的 Simulink/Simscape baseline，主模型为 `simulation/simulink_baseline/model/simulate/two_legs/source.slx`；当前主机可启动 MATLAB R2024b。
- 已有：`docs/models/simulink_mpc_wm_wbc_baseline.md` 已给出控制器层的高层约定：X 前向、Y 侧向、Z 向上，状态中的 position/linear velocity 在 world frame 表达，interaction wrench 在 body-aligned controller frame 表达。
- 已有：当前源码暴露了不能只靠高层说明推断的细节：`controller_attitude_kinematics.m` 声明 Simscape 物理轴为 X 前向、Y 竖直、Z 侧向，并使用 `R = Ry(yaw)*Rz(pitch)*Rx(roll)`；`spatial_two_leg_qp_core.m` 对平移状态使用 `[1,3,2]` 重排；`turning_world_reference.m` 中正 yaw 朝现有 world 约定的负侧向轴转动。
- 已有：`test_wheel_position_coordinate_contract.m` 已冻结 wheel-center 相对 base-forward 的 `xi` 语义，并证明 terrain contact frame 的旋转不得重新定义 `xi`。
- 已有：旧 MuJoCo 导入模型 `simulation/mujoco/model/wheel_leg.xml` 包含嵌套 body frame、局部 `euler`、joint axis、`base_frame` site、accelerometer/gyro/framequat 和 joint sensors，可利用相对 frame 表达所需观测。
- 缺少：从 `source.slx` 的真实 block frame 连接、mask 参数和 MATLAB/Simscape 文档导出的逐信号坐标清单；当前很多传感量看起来在固定 world frame 表达，但尚未形成可审查清单。
- 缺少：world/body/controller/sensor/joint/contact frame 的精确定义、手性、原点、旋转方向、四元数顺序、角速度表达 frame、重力项语义和跨系统变换矩阵。
- 缺少：旧 MuJoCo 坐标与 Simulink 控制语义的差异审计。当前 MJCF 仍有 `gravity="0 0 0"`、base-to-world weld、无显式方向的 `base_frame` site，以及依赖父 body `euler` 的 joint axis，不能视为已批准坐标契约。
- 缺少：MuJoCo/Controller/真机可复用的坐标契约测试和人工正方向核对表。当前默认 Python 环境没有 `mujoco` 包，动态 MuJoCo 检查入口需在执行时明确建立或绑定正式运行环境。

## Scope

- 阅读安装版本对应的 MathWorks R2024b 文档，并以只读方式检查 `source.slx`、相关 mask、Rigid Transform、Joint、Transform Sensor/IMU 类 block 的 frame 端口、测量 frame 和输出参数。
- 建立 Simulink/Simscape 坐标证据清单：每个被 Controller 使用或计划映射到 RobotState 的量都记录源 block、参考 frame、表达 frame、原点、轴向、单位、顺序和符号。
- 审计旧 MuJoCo MJCF 的 world/body/site/joint/sensor 层级、局部 pose、joint axis、零位和左右镜像关系，区分“导入事实”和“批准语义”。
- 冻结一份跨系统 canonical contract，至少覆盖 world、base/body、controller、IMU/sensor、左右 joint、wheel rolling 和 contact frame，以及 pose/twist/acceleration/wrench 的变换规则。
- 对状态与命令边界补齐最小必要的单位、字段顺序、左右顺序、joint 零位/正方向和 quaternion/Euler 表达约定；精确 ROS2 消息 schema 与传输时序仍留给后续 Phase。
- 允许在 MuJoCo 内部保留原生、局部或任务专用 body/site frame；通过命名、注释、辅助 site/frame 和 Adapter 映射把它们转换到 canonical contract，而不是强迫 MJCF 复刻 Simscape 中难用的 world-fixed 传感器连接方式。
- 建立可重复的方向性验证：单位轴、90° 姿态、joint 正向微扰、左右镜像、wheel rolling、IMU orientation/angular velocity/acceleration 和 wrench 变换。
- 对无法安全自动修改的 Simulink GUI、CAD 导出或 MuJoCo 模型操作，先输出精确到对象、字段、目标值和验证截图/导出物的用户操作步骤；用户完成后由 Codex 读取结果并做最终技术验证。

## Out of Scope

- 修改 NMPC、WBC、reference、contact 或 plant 控制算法来适配坐标错误。
- 校准质量、COM、惯量、摩擦、接触、执行器力矩映射、滤波、延迟或噪声。
- 建立完整 MuJoCo Adapter、ROS2 公共消息、STM32 生产协议或正式时间同步语义。
- 用闭环“看起来能跑”替代坐标、符号、零位和变换的直接验证。
- 为追求数组外观一致而消除 MuJoCo 的局部 frame、site 或相对坐标能力。
- 未经 decision gate 直接旋转 CAD mesh、重设 joint 零位、改 joint axis 或覆盖旧 MuJoCo 模型。
- 在 discovery 阶段保存或重构 `source.slx`；若确需模型修改，先建立候选副本或明确的受控修改任务。

## Frozen Decisions

- 已验证 Simulink baseline 的物理与控制行为是迁移语义基准；不得为了迎合旧 MuJoCo 导入坐标而静默改变其状态、输入、方向或符号。
- “统一坐标系”指跨系统同名物理量具有同一 canonical 语义，不要求 Simscape 和 MuJoCo 的每个内部 frame 使用相同原生轴或父子结构。
- MuJoCo 可以使用相对 body/site frame 生成观测和任务量，但 Adapter 输出必须转换为 canonical contract，并在文档中记录 `source frame -> transform -> canonical frame`。
- 每个跨 frame 的 vector、rotation、twist、acceleration 和 wrench 必须注明参考对象、表达 frame、变换方向和单位；禁止无名称的轴交换、隐式负号或仅靠数组下标表达语义。
- 旧 `wheel_leg.xml` 中的 frame、joint axis、零位和 sensor 定义只是待审计候选，不是批准的迁移契约。
- 坐标 discovery 优先使用只读 API 和脚本，不修改冻结 baseline；需要 GUI/CAD/人工操作时必须提供可复核步骤，用户操作结果仍需 Codex 验证。
- 依赖真实方向或安装姿态的结论必须读取模型、仿真或实物证据；没有证据时保留 decision gate，不猜测左右、正负或 IMU 安装方向。

## Open Questions / Decision Gates

- DG01：Simscape physical world、控制器 world 与后续 canonical world 的精确轴映射、正方向和手性是什么；现有 `[1,3,2]` 重排是否还伴随符号变换，必须由 `source.slx`、state reconstruction 和方向性测试共同确定。
- DG02：base/body canonical frame 的原点选在 base reference、几何中心、COM 还是真实 IMU 安装点；姿态描述的是哪个 frame 相对哪个 frame。
- DG03：rotation 使用 active 还是 passive 解释，矩阵记号、乘法顺序、Euler 顺序、quaternion 元素顺序/符号连续性如何定义；continuous yaw 如何从 quaternion 得到并保持 baseline 行为。
- DG04：linear/angular velocity、linear/angular acceleration 分别在哪个 frame 表达；IMU accelerometer 是否含重力/比力，gyro 属于 sensor frame 还是 body frame。
- DG05：左右 hip/knee/wheel 的 joint 顺序、零位、正方向、wheel rolling 正方向与 torque 正方向如何从 Simulink、旧 MJCF 和真机 encoder 对齐；辅助闭环 joint 是否进入公共接口。
- DG06：MuJoCo 是否采用其原生 Z-up world 与局部 site，再在 Adapter 映射到 canonical contract。该方案为首选候选，但必须在 DG01–DG05 和测试证据关闭后冻结。
- DG07：旧 MJCF 的 nested `euler`/mesh 导入方向能否仅通过 XML frame/site 与 Adapter 修正；若必须回到 CAD/GUI 重导出，Codex 需先给出精确用户操作单和预期验证结果。
- DG08：本 Phase 的 MuJoCo 动态方向性测试使用何种正式运行环境；不得把当前缺失的默认 Python `mujoco` 包静默替换成未记录环境。

## Interfaces and Compatibility

- 输入：`source.slx` 的 frame 连接与 block 参数、MATLAB 控制源码、旧 `wheel_leg.xml`、后续 MuJoCo qpos/qvel/sensor data，以及必要的真机 encoder/IMU 人工方向观测；单位按来源记录，不预先假定已统一。
- 输出：`docs/models/coordinate_frame_contract.md`、Simulink frame manifest、MuJoCo frame audit/mapping、canonical 状态/命令字段表、自动测试和用户人工核对表。
- 必须保持：Phase 01 baseline 的控制行为、16-state/12-input 语义、continuous yaw、wheel-center-relative `xi`、canonical differential sign、left-before-right 与 `W* = W_mpc + slack` 符号约定。
- 允许改变：新增只读检查脚本、测试、文档、MJCF 注释/命名/辅助 site 或 frame，以及明确位于 Adapter 边界的坐标转换；任何影响 joint axis、零位、mesh pose 或已验证算法行为的改变必须单独过 decision gate。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| T01 | 固结权威输入与坐标术语 | Phase 01、R2024b 文档、baseline 说明、旧 MJCF | 本 PLAN 的证据清单；frame/transform 记号草案 | 来源优先级与每项待决问题人工审查 | done |
| T02 | 只读提取 Simulink/Simscape frame 与传感器语义 | `source.slx`、mask/脚本、MathWorks R2024b 文档 | 可重复的 MATLAB 检查脚本、Simulink frame manifest、不可自动读取项清单 | 重开模型后重复生成一致；关键 Controller 输入无未标 frame | done |
| T03 | 审计旧 MuJoCo 坐标与关节/传感器定义 | `wheel_leg.xml`、scene、mesh 导入层级 | current-vs-intended 表、body/site/joint/sensor frame tree、风险项 | XML 结构检查；左右镜像、父子 pose、axis/zero/sensor 逐项复核 | done |
| T04 | 关闭 canonical contract 技术决策 | T02、T03、现有控制源码与测试 | `docs/models/coordinate_frame_contract.md`；DG01–DG06 决策记录 | 单位轴、旋转、twist/acceleration/wrench 例子可手算互逆；无隐式 swizzle | doing |
| T05 | 落地 MuJoCo/Adapter 映射与备注 | T04 契约、旧 MJCF | 明确命名的辅助 site/frame、MJCF 注释或独立 mapping；必要的候选修改 | native -> canonical -> native round-trip；原几何/算法未被意外改动 | todo |
| T06 | 建立跨系统方向性测试 | T02–T05 | Simulink 与 MuJoCo 坐标 contract tests、冻结测试姿态与预期符号 | 单位轴、90°、joint 正向微扰、左右镜像、rolling、IMU 和 wrench 全部通过 | todo |
| T07 | 完成用户操作 checkpoint 与真机低风险符号核对 | DG05、DG07、DG08 | 精确操作单、截图/导出/日志入口、`MuJoCo q>0 = Real q>0 = Controller q>0` 核对表 | Codex 复查操作产物；未验证项不得标 PASS | todo |
| T08 | 审查、更新入口并交接后续 Phase | T01–T07 证据 | REVIEW；ROADMAP/Phase 索引；Phase 03/04/06 的输入链接 | REVIEW 无 blocking finding；仅 PASS 后创建 RECORD | todo |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Automated

- `matlab -batch "cd('D:\Workspace\W_L_ws\simulation\simulink_baseline'); open_proformance_test(false); <run frame inspector>; <run coordinate contract tests>"`：在 MATLAB R2024b 中只读提取 source 模型，关键 frame/measurement 参数齐全，方向性断言通过，且不保存修改 `source.slx`。
- PowerShell XML parse + Phase 内 MJCF 审计脚本：`wheel_leg.xml` 可解析，所有公共 joint/site/sensor 名称唯一，父子 frame、axis、zero 和 canonical mapping 无缺项。
- 在 T06 冻结的 MuJoCo 正式环境运行坐标测试：native/canonical round-trip 在数值容差内；单位轴、90° 姿态、joint 微扰、base sensor 和 wheel rolling 输出方向与 contract 一致。
- `git diff --check`：Phase 文档、测试和注释无 whitespace 错误；`git diff -- simulation/simulink_baseline/model/simulate/two_legs/source.slx` 在只读 discovery 后为空。

### Manual / Evidence

- 在 Simulink 中对 inspector 无法读取的 masked block/frame 连接，按 T07 操作单逐项截图或导出参数；记录 block path、端口、选项和 MATLAB release。
- 在 MuJoCo viewer 中显示 canonical base、IMU、joint 和 wheel/contact 辅助 frame；从至少三个视角核对轴向和左右镜像，不以 mesh 外观替代数值检查。
- 对每个真实驱动关节做断能或受限低风险慢速方向核对，记录 encoder q/dq、命令 torque 正方向和物理运动方向；不在本 Phase 调增益或做动力学辨识。
- 若需要 CAD/GUI 重导出，由用户按 Codex 给出的精确操作单执行；Codex 检查导出 XML/mesh pose、数值变换和回归结果后才能关闭 DG07。

## Acceptance Criteria

- [ ] Scope 内交付物完成。
- [ ] `docs/models/coordinate_frame_contract.md` 对每个公共 state/command 字段给出量、单位、参考 frame、表达 frame、原点、顺序和正方向。
- [ ] Simulink frame manifest 覆盖 Controller 实际消费的关键传感/状态量，并能追溯到 `source.slx` block path、参数或控制源码。
- [ ] MuJoCo 的每个公共 body/site/joint/sensor 都能追溯到 canonical mapping；内部相对 frame 有明确命名和备注，不要求复制 Simscape 的 world-fixed 用法。
- [ ] 单位轴、90°、joint 正向微扰、左右镜像、wheel rolling、IMU 和 wrench 方向性测试有真实结果且通过冻结判据。
- [ ] 自动验证通过并记录真实输出；必要的用户 GUI/CAD/真机步骤均有可复核证据。
- [ ] `source.slx` 的 validated behavior 未因 discovery 被改变；任何受控模型修改都有 decision gate 和回归证据。
- [ ] DG01–DG08 全部关闭，或范围外问题明确转入 Phase 03/04/06 且不阻塞本 Phase 的语义放行。
- [ ] 接口和文档与实现一致，REVIEW 无 blocking finding。

## Execution Notes

按任务 ID 记录实际命令、结果、偏差和证据链接；不要建立第二份任务状态表。

建立 PLAN 时的 grounding 结论：高层控制器轴定义、Simscape 物理轴、WBC `[1,3,2]` 重排、positive-yaw 行为和旧 MJCF 局部 frame 已确认存在；这些事实足以定义调查边界，但不足以提前关闭 DG01–DG08。Graphify 只用于定位历史文档关系，最终坐标结论必须以当前源码、模型和真实验证为准。

- T01：查阅 MathWorks R2024b 6-DOF Joint、Revolute Joint 与 quaternion measurement 官方文档，以及 MuJoCo XML/sensor 官方文档；在 `docs/models/coordinate_frame_contract.md` 固结 `{S}`、`{C_fields}`、`{M}` 和 `R_A_from_B` 记号。关键结论是 `[1,3,2]` 为字段排列而非 rotation。
- T02：新增并两次运行 `tools/maintenance/inspect_simulink_frames.m`。第二次清单覆盖 438 blocks 中 111 个 frame/physical 相关 blocks，记录参数、ports、立即连接和 `PortConnectivity`；`source.slx` SHA256 检查前后相同，Dirty `off -> off`。证据见 `evidence/simulink_frame_manifest.json` 与 `evidence/simulink_frame_audit.md`。
- T03：新增并运行 `tools/maintenance/audit_mujoco_frames.ps1`；静态解析得到 11 bodies、10 joints、5 sites、19 sensors、0 个重复公共名称，并确认 base world weld、局部 joint axes、IMU site 和两处 gravity 声明。证据见 `evidence/mujoco_frame_manifest.json` 与 `evidence/mujoco_frame_audit.md`。
- T04：已冻结 Simscape canonical physical `{S}` 为 X 前、Y 上、Z 右；保留 Controller `[前、右、上]` 兼容字段顺序但禁止将其当作 frame。MuJoCo `{M}` 映射、joint sign、IMU acceleration 与 quaternion direction 仍等待动态测试，未提前关闭。
- T06/DG08 前置：发现已有 `conda:mujoco` 空环境后，在该专用环境安装并冻结 MuJoCo 3.12.0，新增 `simulation/mujoco/environment.yml` 与 `audit_mujoco_runtime.py`。旧 scene 成功编译为 nq=17、nv=16、nu=0、sensor width=26；compiled gravity 为 `[0,0,0]`。运行时 freefall/framequat/gyro probes 通过其冻结预期，证据见 `evidence/mujoco_runtime_manifest.json`。
- T06 代数子集：`matlab -batch "... test_coordinate_frame_contract"` PASS；确认 Controller pack permutation det=-1、候选 `R_S_from_M` det=+1、正 yaw 将前向转到物理 -Z/Controller 负侧向。该结果不覆盖 joint torque、rolling contact 或真实 IMU 安装。
- T07：已创建 `USER_CHECKPOINT.md`，把剩余 GUI/真机项目缩减为 Simulink frame 截图、MuJoCo 三视图轴核对和 6 关节/IMU 低风险符号表。

## Blockers

自动工作当前没有环境 blocker；DG08 已由 `conda:mujoco` + `simulation/mujoco/environment.yml` 关闭。完整 Phase PASS 仍等待 `USER_CHECKPOINT.md` 的 GUI/真机证据，并受以下真实模型边界约束：compiled gravity 为零、`nu=0`、base 被 world weld。这些 finding 不在本坐标 discovery 中静默修正。
