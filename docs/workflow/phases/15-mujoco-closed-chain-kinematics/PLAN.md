# Phase 15: MuJoCo 完整闭链运动学与 Jacobian 验证 — PLAN

Status: `complete`

## Goal

在不连接真机、不改变 Controller 公共接口的前提下，证明当前 nominal MuJoCo 五刚体腿闭链在冻结工作域内具有可重复的装配分支、被动关节解、轮心/名义接触点位姿和约束降维 Jacobian，并形成可对未来 SolidWorks revision 与 identified plant profile 原样重跑的非覆盖验证入口。

## Current State

- 已有：Phase 14 已冻结 MuJoCo 3.7.0、完整 contact-free 双腿 fixture、固定基座五刚体单腿 fixture、7 个姿态样本、模型/配置 hash、约束 rank 与内部动力学基线。
- 已有：单腿 fixture 包含 hip/knee/wheel 三个驱动关节、connect1/connect2 两个被动关节和一条 rank-2 closure；完整双腿模型具有左右命名 closure。
- 已有：当前 `reference_leg()` 与 `check_kinematics()` 对 hip→knee→wheel 串联支链执行独立 FK/解析 Jacobian ↔ MuJoCo body/Jacobian 对照。
- 缺少：给定驱动角后的被动关节求解、连续装配分支、两条支链 closure pose 对照、约束降维 Jacobian、中心有限差分、速度映射、虚功/功率一致性、奇异位形与工作域证据。
- 缺少：当前模型没有稳定命名的几何接触 site；Simulink 的 `0.08 m` wheel radius 是简化仿真假设，不能未经 grounding 直接升级为当前 MuJoCo collision mesh 的事实。
- 证据差异：Phase 14 PLAN/T03 的文字包含 finite-difference/速度预测，但当前正式 runner 没有实现这两项。本 Phase 以当前源码和结果 JSON 为事实来源补齐，不回写或覆盖 Phase 14 历史 evidence。
- 路线：真机相关 Phase 05 保持 `blocked`；本 Phase 是总体 simulation-only 路线的下一项，不关闭任何 MuJoCo–real gate。

## Scope

- Ground 当前左右五刚体闭链的 joint/body/site、平面、轴、固定变换、collision mesh、轮心和闭链端点，形成机器可读 geometry manifest。
- 冻结 current nominal plant profile、模型 hash、MuJoCo/solver 版本、独立/被动坐标分区、装配分支、采样工作域、奇异性指标和验证阈值。
- 给定 hip/knee/wheel 后确定性求解 connect1/connect2；使用 nominal 零位分支和邻域连续延拓，显式拒绝不可达、未收敛、分支跳变和近奇异样本。
- 使用两条独立支链计算 closure 端点并对比 MuJoCo site pose；验证左右镜像关系、轮心位姿和 wheel rolling direction。
- 由 equality Jacobian 构造从驱动速度到完整五关节速度的约束切空间映射，得到 wheel center 与名义接触点的 reduced Jacobian。
- 对解析/约束降维 Jacobian、MuJoCo Jacobian和“每次扰动后重新求被动关节”的中心有限差分做三方比较；姿态差使用 SO(3) 对数或等价稳定表示。
- 验证 `v = J dq`、角速度映射、关节正向微扰方向，以及 `tau = J^T wrench` 的虚功/功率一致性。
- 扫描冻结工作域，记录 closure residual、被动解、Jacobian rank/奇异值/条件数、分支连续性、最差样本与明确排除域；不以少量典型姿态代表整个工作域。
- 建立 profile-driven、headless、确定性的 runner、config、manifest、JSON/CSV 和报告入口；未来模型 revision/identified profile 通过显式参数选择复用。

## Out of Scope

- 任何真机上电、板级联调、encoder/IMU/Load Cell 采集、执行器辨识或真机参数结论。
- 修改或标定 mass、COM、inertia、friction、armature、actuator torque mapping、delay 或 contact material 参数。
- 轮地接触求解器、摩擦/滑移、接触力保真度、floating-base 落地或接触动力学验证。
- Joint PD、重力补偿、站立、WBC、NMPC 或 Controller 算法实现。
- 为适配测试而改变 canonical FLU、关节顺序、符号、RobotState/TorqueCommand schema 或 Adapter 映射。
- 把 nominal 接触点、collision mesh 或 Phase PASS 描述成真机几何/接触已经校准。
- 原地覆盖 `wheel_leg.xml` nominal baseline、Phase 14 evidence 或任何已完成 Phase 的 PLAN/REVIEW/RECORD。

## Frozen Decisions

- 本 Phase 的验收对象是 Phase 14 当前 nominal model revision；未来 SolidWorks 或 identified revision 只复用入口并产生新 run/新证据，不静默改变本 Phase 的验收输入。
- 单腿广义坐标按模型地址读取；独立坐标固定为 `[hip, knee, wheel]`，被动坐标固定为 `[connect1, connect2]`。不得用 actuator 数量或 XML 顺序猜测地址。
- 被动解以 closure site 三维位置残差为方程；有效约束秩为 2。求解器必须使用冻结初值/容差/最大迭代，并以 nominal 零位分支的连续延拓选择装配模式，不按单点最小残差任意切换分支。
- 约束降维使用 `Jc * dq = 0`。对 active/passive 分块后构造完整速度映射 `S`，并以 `J_reduced = J_object * S` 作为三驱动坐标 Jacobian；每个样本必须检查 passive block rank 和条件数。
- 有限差分扰动只施加于独立坐标；每个 `q±epsilon` 都必须重新求解被动坐标。直接固定被动角扰动不是闭链 reduced Jacobian 的有效参考。
- wheel center 与名义 contact point 分开：wheel center 是稳定刚体点；名义 contact point 是由 profile 中有 provenance 的轮几何、局部轴与外部法向定义的可微测试点，不等同于 MuJoCo 瞬时 contact manifold。
- 接触点姿态不作为物理唯一量；验证其位置、线速度/Jacobian、rolling/tangent/normal 方向。轮刚体姿态单独验证。
- 正式工作域、finite-difference epsilon、pose/Jacobian/closure/power 容差和 singularity exclusion 必须先写入版本化 config 再正式运行，不得看见结果后放宽。
- 所有结果目录和 evidence 追加写入；manifest 至少记录 model revision/hash、profile、MuJoCo 版本、runner commit/hash、solver、seed、采样域、阈值和输入 hash。已有目录非空时 runner 必须拒绝覆盖，除非显式选择新的 run ID。
- 当前 canonical 映射保持 `q_C=-q_M+b`、`dq_C=-dq_M`、`tau_M=-tau_C`；Phase 15 的原生/reduced 坐标结果必须明确标注所在空间。

## Open Questions / Decision Gates

- **DG01 / CLOSED / EVIDENCE — contact-point profile：** compiled collision mesh 径向最大值约 `0.05012 m`，profile 冻结 `0.05 m` nominal radius 和局部 contact point `[0.05,0,0] m`；与 Simulink `0.08 m` 分离，provenance/model hash 见 geometry manifest。
- **DG02 / CLOSED / EVIDENCE — assembly branch/workspace：** 左/右 nominal 被动角关系、零位 continuation、正反路径和 210 个工作域样本全部通过；最大 closure `3.2474e-15 m`，无隐藏失败或排除样本。
- **DG03 / CLOSED / EVIDENCE — singularity gate：** 预冻结 `min singular >= 0.005`、`condition <= 40`；正式最坏值 `0.0073709 / 30.1993`，全部工作域样本通过。
- **DG04 / CLOSED / CODEX_DECISION — Jacobian reference：** 三方验证固定为约束降维解析/MuJoCo、重求闭链的中心有限差分，以及速度/虚功一致性；任意两方相互复用同一结果不能算独立证据。
- **DG05 / CLOSED / CODEX_DECISION — versioning：** nominal、未来 SolidWorks revision 与 identified profile 使用相同 schema/runner、不同 manifest/run ID；新结果追加并通过 `supersedes`/comparison 关系连接，不覆盖旧证据。
- **DG06 / CLOSED / EVIDENCE — quantitative thresholds：** 正式阈值在 formal run 前写入 `phase15_nominal.json`；三档 epsilon 与全部 gate 通过，formal run 后未放宽。

## Interfaces and Compatibility

- 输入：显式 plant profile、MuJoCo scene/model、model revision/hash、驱动坐标采样/速度/wrench、solver/epsilon/seed、工作域和阈值 config。
- 输出：geometry/profile manifest、passive solution/workspace map、closure/pose/Jacobian/velocity/virtual-work JSON/CSV、最差样本、排除域、运行摘要和 REVIEW evidence。
- 坐标：SI 单位；角度 `rad`、位置 `m`、速度 `m/s` 与 `rad/s`、wrench `N/N·m`、关节力矩 `N·m`；native 与 canonical 字段分开命名。
- 必须保持：Phase 02–04 坐标/接口/Adapter 契约、Phase 14 nominal assets/evidence、Phase 05 frozen 状态以及当前产品运行默认行为。
- 允许改变：新增 Phase 15 fixture/site、profile/config、实验 runner、分析工具和 evidence；若发现当前模型 bug，只能在记录 finding、确认影响范围并完成 Phase 04/14 回归后修改。
- 复用要求：runner 必须支持通过参数选择 model/profile/output run ID，不通过复制脚本或编辑源码切换 nominal/revision/identified plant。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| T01 | Ground 闭链、轮心与 contact-point geometry，关闭 DG01 | `wheel_leg.xml`、single-leg fixture、compiled mesh/model、Phase 02/04/14 evidence | geometry manifest、对象/坐标/半径 provenance 与差异记录 | 所有 joint/body/site/geom 按名字和地址解析；单位/轴/左右镜像/contact-point 定义可追溯 | done |
| T02 | 冻结 nominal profile、fixture/config 与非覆盖目录契约 | T01、Phase 14 config/manifest、总体两轮路线 | versioned plant profile、Phase 15 config、fixture invariants、run manifest schema | model/config hash 固定；旧 evidence 未修改；非空输出目录覆盖测试明确失败 | done |
| T03 | 实现被动关节求解与连续装配分支，关闭 DG02 | T01/T02、closure sites、active/passive partition | deterministic passive solver、continuation sweep、workspace/branch map | nominal 零位与往返路径收敛；closure residual、迭代、分支 ID、不可达样本完整记录 | done |
| T04 | 建立完整闭链独立 FK 与左右镜像验证 | T03、两条支链固定变换 | closure 两支链 pose、wheel center/rigid-body pose reference | 两支链与 MuJoCo site/body 输出在冻结容差内；左右镜像和 rolling direction 通过 | done |
| T05 | 构造约束切空间与 reduced Jacobian | T03/T04、equality/object Jacobian | `S(q)`、passive velocity map、wheel center/contact-point reduced Jacobian | `Jc*S` 残差、rank、奇异值和条件数满足 DG03；全量最差样本输出 | done |
| T06 | 完成三方 Jacobian、速度和方向验证，关闭 DG04/DG06 | T05、冻结 epsilon/速度样本 | analytic/MuJoCo/finite-difference comparison 与 epsilon convergence | 每次扰动重求 closure；位置/旋转/速度误差通过预冻结阈值 | done |
| T07 | 验证虚功、功率与 wrench→joint torque 映射 | T05/T06、确定性 force/wrench samples | reduced `J^T` mapping、power audit、sign evidence | `tau^T dq` 与 `wrench^T twist` 在冻结阈值内；native/canonical 符号明确 | done |
| T08 | 完成工作域、奇异性与分支鲁棒性 sweep，关闭 DG03 | T03–T07、冻结 grid/seed/boundary | workspace coverage、singularity/exclusion map、branch continuity report | 全部声明工作域样本有状态；失败/排除不丢弃；正反路径与同 seed 重跑一致 | done |
| T09 | 固化可复用 runner、日志和跨 revision comparison | T02–T08、非覆盖契约 | headless runner、JSON/CSV schema、profile comparison entrypoint、使用文档 | nominal 重跑确定；替代 profile smoke 不修改算法源码；输出目录和 manifest 可并存 | done |
| T10 | 执行回归、汇总证据并准备 REVIEW | 全部任务 | automated evidence、方法文档、Execution Notes、REVIEW 输入 | Phase 04 coordinate/Adapter 与 Phase 14 suite 不回退；DG01–DG06 关闭，无硬件证据 | done |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Automated

- `./.venv/bin/python tools/maintenance/test_mujoco_coordinate_contract.py`：Phase 02/04 坐标、joint sign、COM frame 和 nominal model contract 继续 PASS。
- `./.venv/bin/python tools/experiments/run_mujoco_internal_dynamics.py`：Phase 14 九项内部动力学基线继续 PASS，旧 evidence 不被覆盖。
- `./.venv/bin/python tools/experiments/run_mujoco_closed_chain_kinematics.py --profile nominal --output-dir data/experiments/<run-id>/raw`：正式闭链工作域、FK、Jacobian、速度、虚功、奇异性与确定性 gate 全部 PASS；实际 run ID 在执行时追加记录。
- `./.venv/bin/python tools/experiments/run_mujoco_closed_chain_kinematics.py --profile nominal`：生成 Phase evidence 入口，包含输入 hash、最差样本和预冻结阈值；存在旧结果时不得静默覆盖。
- `cd ros_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-up-to wheel_leg_mujoco && colcon test --packages-select wheel_leg_mujoco && colcon test-result --verbose`：若模型、Adapter package 或其测试 fixture 有改动，相关 build/test 无失败。

### Manual / Evidence

- Codex 审查 nominal assembly branch、工作域覆盖、被排除样本、最小奇异值、epsilon convergence 和所有 worst-case 指标；不安排用户操作 viewer 或真机。
- 至少抽查 nominal、边界、近奇异和左右镜像样本的两支链 geometry；截图可辅助解释，但不能替代机器可读 pose/residual。
- REVIEW 必须核对 Phase 14 历史 evidence 的 hash/内容未被本 Phase 原地覆盖，并验证两个不同 run ID/profile 的结果可以并存和比较。
- 若 SolidWorks 新 revision 在本 Phase REVIEW 前到达，只作为独立附加 profile run；nominal acceptance 不被替换。是否将新 revision 纳入正式验收需显式记录 scope decision。

## Acceptance Criteria

- [x] T01–T10 完成，DG01–DG06 由预冻结配置和真实 MuJoCo 结果关闭。
- [x] 给定冻结工作域内的 hip/knee/wheel，connect1/connect2 解确定、连续、属于 nominal 装配分支，closure residual 通过阈值。
- [x] 两条闭链支链、左右镜像、轮心位姿、轮刚体姿态及 nominal contact-point 定义均有独立、可追溯验证。
- [x] reduced Jacobian 同时通过约束切空间、MuJoCo、重求闭链中心有限差分三方对照，并通过 `v=Jdq`。
- [x] 虚功/功率映射、joint/contact direction 和 native/canonical sign 在冻结容差内一致。
- [x] 工作域覆盖、不可达域、装配分支和奇异性边界完整输出，不用平均值或删除失败样本掩盖最坏情况。
- [x] runner/config/schema 可对 nominal、未来 SolidWorks revision 和 identified profile 原样复用；切换 profile 不修改算法源码。
- [x] 模型、配置、runner、阈值和输入 hash 进入 manifest；新 run 追加写入，Phase 14 与其他历史 evidence 未覆盖。
- [x] Phase 04/14 回归继续 PASS；无真机操作或 MuJoCo–real 一致性声明。
- [x] 自动结果、方法文档、README/ROADMAP 和实际实现一致，无 blocking finding 后才进入 REVIEW。

## Execution Notes

- 2026-08-25：根据用户要求只制定 Phase 15，不开始实现。Phase 目标从总体 simulation-only 路线细化为完整闭链、contact-point 和 Jacobian 验证；状态保持 `planned`。
- 2026-08-25：Grounding 确认 Phase 14 当前正式 runner 只实现串联支链解析 FK/Jacobian ↔ MuJoCo 对照，未实现其 PLAN 文本曾列出的 finite-difference/速度预测；本 Phase 追加补齐，不改写 Phase 14 历史证据。
- 2026-08-25：冻结 nominal/未来 revision/identified profile 共用 runner 与追加式 evidence 原则；当前 nominal model revision 是本 Phase 唯一必需验收 profile。
- 2026-08-25：用户授权执行 Phase 15；状态切换为 `active`。执行仍限定为 MuJoCo-only，不连接真机、不修改 Phase 14 历史 evidence。
- 2026-08-25：预冻结 nominal `0.05 m` contact radius、210 样本工作域、solver/epsilon/奇异性与误差阈值；首轮发现左右姿态判据误把 reflection 当 SO(3)，修正为“位置镜像、frame 同向”后完整 sweep PASS，未放宽数值阈值。
- 2026-08-25：coordinate contract、Phase 14 regression、Phase 15 formal run、完整重复确定性和 non-overwrite gate 全部通过；进入 REVIEW 后无 blocking finding，创建 RECORD 并完成 Phase。

## Blockers

None. 本 Phase 不依赖真机、STM32、Load Cell、Hardware Adapter 或 identified 参数；DG01/DG02/DG03/DG06 是 Phase 内必须用 MuJoCo/模型证据关闭的 gate。
