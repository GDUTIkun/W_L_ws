# Phase 19: 显式二维 sagittal 简单站立 — PLAN v2

Status: `review`

Supersedes for execution: [`PLAN-v1-2026-08-26-REWORK.md`](PLAN-v1-2026-08-26-REWORK.md)

Preserved failed review: [`REVIEW-v1-2026-08-26-REWORK.md`](REVIEW-v1-2026-08-26-REWORK.md)

## Goal

在不连接真机、不引入 WBC/QP/NMPC 的前提下，从 current nominal `wheel_leg.xml` 可重复派生一个**显式二维 sagittal plant**：base 只保留 world `X` 平移、world `Z` 平移和绕 world `+Y` 的 pitch，自模型层删除 `Y/roll/yaw` 自由度；其余完整多刚体惯量、左右闭链、六个 actuator、actual-wheel collision、floor contact 和 `2 ms / 10 ms / 5-step ZOH` 不变。在该 plant 上重新求零轮扭矩站立平衡点，建立并验证固定腿姿态 + equal common-wheel 四状态反馈，使 `x/pitch/height/leg posture/contact` 在冻结小扰动范围内通过至少 `10 s` simulation-only formal hold。

## Why v2 Exists

- v1 要求 sagittal/common-wheel controller，同时又在完整 3D floating plant 上把 `Y/roll/yaw` 泄漏设为硬门槛，plant 自由度与 controller authority 不匹配。
- v1 pre-freeze 模型可控，但候选闭环谱半径为 `1.0320567 > 1`；完整 plant 只能得到带偏差的有限时间有界运动，不能进入 Core/formal。
- 用户批准按显式二维路线重划。v1 PLAN、REVIEW、profile、script、raw/replay evidence 全部保留，不覆盖、不删除。

## Scope

- 新增可审计的 planar-model generator。输入 authoritative `wheel_leg.xml`，只把 `base_body` 唯一、直接且无属性的 `<freejoint/>` 替换为 `base_x_joint`、`base_z_joint`、`base_pitch_joint`；任何锚点缺失/重复、额外结构差异或 source hash 不匹配均 fail closed。
- generator 输出进入每个 run 自己的新目录；正式 scene 同样由 wrapper 在 run 目录生成并引用该 derived model。仓库不手工维护第二份完整机器人 XML。
- 编译并审计 full-vs-planar 差异：base DOF 从 6 变 3；body/geom/site/inertial/actuator/leg equality/contact/solver/timestep 的名字、数量和数值保持一致；只有 base joint topology、随之变化的 `nq/nv` 和 joint/dof address 允许不同。
- 在 exact planar plant 上重新求解 upright、双轮接触、零速度、左右 wheel torque 都为零的 contact-aware equilibrium；允许在 Phase 15 工作域内选择由当前模型左右差异决定的 side-specific leg reference。
- standing state 固定为 `x_s=[x-x_ref, dx, theta, dtheta]`，来自 canonical `base_control_frame` site position/Jacobian twist/quaternion/world angular velocity；reset 锚定 `x_ref`。
- hip/knee 使用 contact-equilibrium support + fixed-reference PD；wheel position PD 和 wheel gravity项禁用，左右轮只接受完全相等的 common balance torque。
- 从 `2 ms × 5` 的完整 planar contact plant 数值生成 10 ms local `A/B`，冻结 gain、poles、controllability、affine drift、holdout residual 和有效 envelope；runtime Core 不依赖 MuJoCo。
- 预冻结 gate 通过后，扩展 Controller Core 的 opt-in `simple_standing` mode 和现有 Phase 16 deterministic C++ loop；失败则再次 REVIEW=`REWORK`，不继续堆控制层。
- formal 覆盖 equilibrium、正负 pitch/rolling 初值、正负 base-X force/pitch moment、腿姿态扰动、saturation、contact loss、invalid/nonmonotonic、reset/replay、non-overwrite 和 Phase 02/04/14/15/16/17/18 回归。
- 保存 source model、generator、derived model、scene、config、controller、runner 和 outputs hashes，为后续 SolidWorks revision/identified profile 重新派生、重新求 equilibrium/model/gain 提供非覆盖入口。

## Out of Scope

- 真机上电、物理导轨/保护架实验、STM32/树莓派联调、传感器/执行器/接触辨识以及任何 MuJoCo–real 一致性结论。
- 用 penalty force、lateral spring/damper、每步覆写 qpos/qvel、隐藏 weld 或阈值放宽伪造二维约束；二维必须来自模型 DOF topology。
- 完整 3D 自由站立、`Y/roll/yaw` 控制、差分转向、单轮支撑、斜坡、台阶、跌倒恢复和大范围 region-of-attraction 声明。
- Cartesian height/wrench/contact-force controller、inverse dynamics、QP、Weighted WBC、NMPC、积分器、observer、gain scheduling 或在线 MuJoCo linearization。
- 修改 canonical frame/sign/order、公共 `RobotState/TorqueCommand` schema、Phase 18 contact profile、Phase 16 timing、历史 evidence 或 current nominal source model。

## Frozen Decisions

- **Plant authority：** Phase 19 PASS 只对 derived exact-planar current nominal plant 有效，不再称为完整 3D floating standing PASS。完整 3D plant 的零控制/contact authority 仍来自 Phase 18。
- **Exact planar topology：** base generalized coordinates严格为 `[x_world, z_world, pitch_about_world_+Y]`；不允许 `Y/roll/yaw` DOF，也不允许用外力近似约束。generator 必须从 source XML 每次派生，禁止手工同步两份完整模型。
- **Preserved physics：** 除 base joint topology 外，mass/inertia/COM、mesh、joint axis、闭链 equality、actuator、contact、gravity、solver、timestep 全部与 source/profile一致；structural diff auditor 是 formal 前置 gate。
- **Timing：** physics `0.002 s`，control `0.010 s`，5-step ZOH；state 只在 control tick 采样，command 在随后 5 个 physics step 不变。
- **Equilibrium：** upright `theta=0`、双轮接触、零速度、左右 wheel torque 精确为零。必须验证 compliant closure、constraint/contact force、normal load、qacc/generalized residual、finite 和 reset replay；不允许用控制器稳态偏差替代 equilibrium。current nominal CAD/MuJoCo 左右不严格镜像，因此冻结 side-specific active references，并限制左右差异，不用“强制同值”制造残余加速度。
- **State/sign：** canonical world FLU，pitch 遵守 world `+Y` 右手定则；`x/dx/theta/dtheta` 由 Adapter-compatible site oracle提取。正 canonical wheel rotation/no-slip `+X` 继续继承 Phase 15。
- **Controller：** 左右 leg 分别使用冻结的 fixed posture `support_eq + PD`；wheel 为 equal common torque `tau_common=-K*x_s`。`z` 仅由几何、腿姿态和接触间接维持并作为硬指标，不加入独立 height task。
- **Pre-freeze first：** equilibrium、4-state model、stable poles、正负 holdout 和 full-planar-plant exploratory recovery 全通过后，才允许实现 Core/C++ formal chain。
- **Fail closed：** torque、pitch、height、joint velocity、contact、finite、sample time 任一越过冻结 envelope 时输出零并锁存到 reset。
- **Non-overwrite：** v1 失败 evidence 是永久历史；v2 exploratory、formal、replay、未来 CAD/identified runs 均使用新目录和 `supersedes` 链。
- **3D boundary：** 完整 3D standing 是后续独立 Phase，至少需要 roll/yaw/lateral sensing、control authority 和验证矩阵；不得从本 Phase 的 2D PASS 推断自由真机或 3D MuJoCo 可站立。

## Decision Gates

- **DG19-01 / CLOSED / USER+CODEX — route：** 采用 exact 2D sagittal plant；完整 3D standing 顺延。
- **DG19-02 / CLOSED / CODE+EVIDENCE — derived-model fidelity：** generator、structural diff auditor 和 exact replay 证明除 base DOF topology 外 current nominal physics 未改变。
- **DG19-03 / CLOSED / EVIDENCE — equilibrium：** compliant-contact/equality solve 找到 zero-wheel-torque upright equilibrium；qacc、广义力、双轮载荷、closure、one-step drift、finite 和 exact replay 全通过。
- **DG19-04 / CLOSED / EVIDENCE — state/sign：** site/quaternion/Jacobian、finite difference、wheel rolling 和 canonical/native torque 方向一致，primary/replay exact。
- **DG19-05 / OPEN / BLOCKING EVIDENCE — local controller：** reset-local 4-state model 虽 rank `4` 且候选谱半径 `<1`，完整 26-state sampled plant 谱半径 `1.767146`，nonlinear holdout 失败；不得进入 Core。
- **DG19-06 / OPEN / EVIDENCE — formal envelope：** gains、limits、disturbances 和 thresholds 在 holdout 前冻结；全部 10 s cases 通过。
- **DG19-07 / OPEN / EVIDENCE — runtime/reuse：** Core/Adapter/C++ loop、fault/reset/replay、non-overwrite、历史回归和 new-revision dry-run 全通过。
- **DG19-08 / CLOSED / SCOPE — claims：** 结论只限 current nominal exact-planar simulation，不是完整 3D、WBC 或真机证据。

## Tasks

| ID | Task | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- |
| P19V2-T01 | 归档 v1 并重划 roadmap/docs | v1 PLAN/REVIEW保留，v2 PLAN、route、phase index一致 | 旧 raw/replay hashes不变；无 RECORD | done |
| P19V2-T02 | 实现 source→planar model generator 与 diff auditor | generator、generated scene/model manifest、tests | 唯一 freejoint替换；允许差异白名单外为零；DG19-02关闭 | done |
| P19V2-T03 | 求解和验证 exact-planar contact equilibrium | equilibrium solver/profile/report | zero wheel torque、qacc/constraint/contact/closure/load/reset过阈值；DG19-03关闭 | done |
| P19V2-T04 | 冻结 canonical state/sign/rolling contract | evaluator与正负 oracle | site/Jacobian/finite difference/rolling/native-canonical一致；DG19-04关闭 | done |
| P19V2-T05 | 生成 local A/B 和稳定 common-wheel gain | model/gain/fit/holdout evidence | reset-local PASS，但 full 26-state/nonlinear gate FAIL；DG19-05未关闭 | blocked |
| P19V2-T06 | 实现 Core standing mode 与 C++ planar loop | opt-in Core config/diagnostics、runner extension、tests | equal wheel torque、leg decomposition、ZOH、trip/reset、旧 mode兼容 | blocked |
| P19V2-T07 | 建立 v2 wrapper/profile/case matrix | non-overwrite raw/summary/manifest | Python只编排/评价；hash/schema/cases/threshold完整 | blocked |
| P19V2-T08 | 冻结 exploratory envelope 并执行 formal | 新 exploratory/formal evidence | 所有 10 s holdout/fault cases通过；DG19-06关闭 | blocked |
| P19V2-T09 | replay、历史回归与 revision reuse audit | fresh-process、Phase 02/04/14–18 regression、dry-run | determinism/compatibility/reuse通过；DG19-07关闭 | blocked |
| P19V2-T10 | REVIEW | 新 `REVIEW.md`；仅 PASS 后创建 RECORD | 当前 Verdict=`REWORK`，无 RECORD | done |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Pre-freeze

- generator unit tests：唯一 freejoint锚点、三关节 name/type/axis/order、重复运行 exact、非目标差异 fail closed。
- full-vs-planar compiled audit：body/geom/site/inertial/active/passive joints/actuators/equalities/contact/solver/timestep exact；只允许 `nq/nv`、base joint set/address变化。
- equilibrium：双轮 contact、零 wheel torque、有限且 side-specific 的 leg support、compliant closure、normal load、`qacc`/generalized residual、one-step drift 和 fresh reset replay。
- local model：10 ms `A/B`、rank、affine drift、poles、中心差分步长收敛、未参与设计的正负 one/multi-step holdout，以及完整 planar contact plant recovery。

### Formal

- `colcon build/test` 覆盖 Core、ROS compatibility、Adapter 和 deterministic runner。
- 至少 `10 s` nominal hold、正负 pitch/rolling、正负 X-force/pitch-moment、leg posture、saturation/contact-loss/invalid/nonmonotonic/reset cases。
- 每个 case 检查 pitch、x drift/recovery、height、leg error、双轮 contact、normal load symmetry、slip/penetration、closure、torque/velocity/saturation、finite 和 ZOH。
- primary/replay 使用两个新目录；规范化 outputs exact 或在预冻结 tolerance 内一致；非空目录在仿真前拒绝。
- Phase 02/04 coordinate/Adapter，Phase 14/15 dynamics/closed-chain，Phase 16 loop，Phase 17 fixed-base PD/gravity，Phase 18 full-3D contact/free-flight 全部回归。

## Acceptance Criteria

- [ ] derived model 的唯一物理差异是 base 从 freejoint 变为 exact `X/Z/pitch` 三自由度，且生成/审计/manifest 可跨 CAD revision 重跑。
- [ ] zero-wheel-torque upright equilibrium 通过静力、contact、closure、reset 和 finite gates。
- [ ] canonical state/sign/rolling 与 Adapter/Phase 15 一致，公共 message 不变。
- [ ] 4-state local model、gain、poles、drift和holdout通过，full planar contact plant 在冻结正负小扰动范围恢复。
- [ ] Core standing mode opt-in、default-zero/Phase17兼容、左右轮力矩完全相等、故障 fail closed。
- [ ] 全部 formal cases 至少运行 `10 s` 并通过冻结指标；determinism、non-overwrite和历史回归通过。
- [ ] v1 REWORK evidence 和所有历史 evidence 未覆盖；v2结论明确限制为 exact-planar current nominal simulation-only。
- [ ] REVIEW=`PASS` 后才创建 RECORD 和把 ROADMAP 标为 complete。

## Execution Notes

- 2026-08-26：用户接受 Codex 推荐，将 Phase 19 从“完整 3D plant 上只做 sagittal control”重划为“exact 2D sagittal plant”。
- 2026-08-26：v1 PLAN/REVIEW 改为带版本文件名的只读历史入口；v1 exploratory primary/replay evidence 原路径保留。
- 2026-08-26：Graphify 仅查询已有图，图缺少最新 Phase 19 REWORK；未执行 extract/update。当前重划依据 live docs、source model 和真实 v1 evidence。
- 2026-08-26：P19V2-T02 PASS。derived model 只改变 base topology；preserved compiled-field、初始 body pose 和 solver/options 最大差异均为 `0`，primary/replay hashes exact。
- 2026-08-26：P19V2-T03 PASS。柔性闭链/接触一致求解得到 `max|qacc|=9.815e-11`、零 wheel torque 和双轮正载荷；current nominal 左右不严格镜像，冻结小幅 side-specific leg references，而非强制相同 native angles。
- 2026-08-26：P19V2-T04 PASS。site/Jacobian/有限差分误差不超过 `1.025e-12`，native 正轮转对应 `+X` rolling，Adapter canonical/native 反号关系明确且 replay exact。
- 2026-08-26：P19V2-T05 pre-freeze REWORK。reset-local 四状态候选谱半径 `0.984789`，但完整 26-state sampled 闭环谱半径 `1.767146` 且五个 nonlinear case 失败；按 gate 未修改 Core。

## Blockers

DG19-05 blocking：必须先归因完整 sampled plant 的 equality/contact/leg 隐藏不稳定模态，并重新通过 full-state linear + nonlinear recovery；在此之前 P19V2-T06–T09 不执行。
