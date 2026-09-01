# 项目 ROADMAP

本文件是阶段状态的唯一总台账。技术细节、任务执行和审查内容写入对应 Phase；本文件只维护阶段顺序、状态、依赖和链接。

## 状态定义

- `planned`：已列入路线，尚未开始。
- `active`：正在设计、实现或验证。
- `review`：实现已停止扩张，等待或正在审查。
- `complete`：REVIEW 为 PASS，RECORD 已完成。
- `blocked`：存在明确阻塞条件，无法继续。
- `cancelled`：因路线变化终止；保留历史产物，不要求 PASS/RECORD，也不代表技术通过。

## 当前总体状态

- 当前唯一正式路线为 `Controller Core → ROS2 → MuJoCo`；current launch 为
  `wheel_leg_mujoco current_weighted_wbc.launch.py`。
- Simulink baseline 只作为冻结算法/数值参考，不属于 runtime，也不再扩展。
- STM32、串口、Hardware Adapter、树莓派部署和所有实物验证已从路线及源码移除。
- Phase 46 保持历史 `review/REWORK`；其 evidence 和数值结论不因路线重置而改写。

## 当前路线决策：MuJoCo-only 与非覆盖

- 当前只推进 MuJoCo simulation。Phase 47 清理并冻结接口；Phase 48 关闭
  Weighted-WBC/QP realization；Phase 49 比较 12X/16X NMPC candidates。
- 任何历史 real/identified/hardware 计划均已退役，不存在“解冻后继续”的隐含分支。
- 每个后续阶段都必须把模型版本、参数 profile、Controller 版本、求解器配置、seed/激励、阈值和输入文件 hash 写入 manifest；运行输出进入新的带日期/模型 ID 的目录。已完成 Phase 的 PLAN/REVIEW/RECORD 和正式 evidence 不原地覆盖，修订通过新 Phase、新 run 或带 `supersedes` 关系的记录追加。
- 当前 `wheel_leg.xml` 与 Phase 14 evidence 作为 nominal baseline 保留。后续 SolidWorks 调整髋部电机或连接件尺寸并重新导出时，必须建立新的模型 revision，保留旧导出和 hash；重新检查 joint/body/site 名称与拓扑、frame/axis/zero offset、closure、collision、mass/COM/inertia，并重跑 Adapter、运动学、内部动力学及Phase20 equilibrium/authority/gain。接口不变时控制与验证入口应直接复用，但不能假定几何、惯量或控制数值自动不变。

## 阶段路线

| 顺序 | 阶段 | 状态 | Phase | 放行条件/证据 |
| --- | --- | --- | --- | --- |
| 01 | 迁入 Simulink 基线与验证入口 | complete | [Phase 01](phases/01-simulink-baseline-import/PLAN.md) | 基线模型、运行方式和当前验证结果可复现 |
| 02 | 坐标系、单位、关节顺序与接口语义 | complete | [Phase 02](phases/02-coordinate-interface-contract/PLAN.md) | FLU canonical、Simscape/MuJoCo 映射、COM frame 与 joint sign 契约通过审查；旧硬件补充 gate 已由 Phase47 退役 |
| 03 | 统一 Robot 接口与 Controller Core 骨架 | complete | [Phase 03](phases/03-robot-interface-controller-core/PLAN.md) | C++ Core、聚合消息、ROS2 wrapper 与 Jazzy pub/sub 测试通过 |
| 04 | MuJoCo 基础模型与 Adapter | complete | [Phase 04](phases/04-mujoco-model-adapter/PLAN.md) | MuJoCo 3.7.0 状态/零力矩闭环、fixed/floating sanity、映射与 fail-safe 通过审查 |
| 05 | MuJoCo 运动学与内部动力学验证 | complete | [Phase 14](phases/14-mujoco-internal-dynamics-validation/PLAN.md) | 不接真机；FK/Jacobian、重力、M(q)、正逆动力学、约束、耦合、能量与开环回放自洽并通过审查 |
| 06 | 完整闭链运动学、接触点与 Jacobian 验证 | complete | [Phase 15](phases/15-mujoco-closed-chain-kinematics/PLAN.md) | 210 样本 nominal 装配分支、被动解、工作域、reduced Jacobian、有限差分、速度与虚功通过 REVIEW；入口可跨 revision 非覆盖复用 |
| 07 | Controller ↔ MuJoCo 确定性闭环运行基线 | complete | [Phase 16](phases/16-controller-mujoco-deterministic-loop/PLAN.md) | 不接真机、不新增控制算法；2 ms/10 ms/5-step fixed loop、双时钟/reset/fail-safe、逐 tick 日志、replay 与非覆盖 gate 通过 REVIEW |
| 08 | nominal Joint PD 与重力补偿 | complete | [Phase 17](phases/17-nominal-joint-pd-gravity-compensation/PLAN.md) | 不接真机；解析 reduced gravity + canonical Joint PD 已通过双 oracle、保持、正负阶跃、限幅、扰动、对称与 replay 审查 |
| 09 | nominal 轮地接触与 floating-base plant 验证 | complete | [Phase 18](phases/18-mujoco-contact-floating-base-plant-validation/PLAN.md) | 不接真机；wheel-only contact、normal/rolling/lateral/friction、零控制 touchdown、base state/reset 已通过 REVIEW，不提前做站立 |
| 10 | exact 2D sagittal 简单站立 | complete | [Phase 19](phases/19-nominal-planar-simple-standing/PLAN.md) | formal-v4 11 个 10 s normal/perturbation + 4 个 fault cases PASS；REVIEW/RECORD 完成，仅限 current nominal exact-planar simulation |
| 11 | nominal 完整 3D 简单站立 | complete | [Phase 20](phases/20-nominal-3d-simple-standing/PLAN.md) | formal-v3 19个10 s normal/perturbation + 6个fault cases PASS；full-3D state/contact/slip/closure、fresh replay、REVIEW/RECORD完成，仅限current nominal simulation |
| 12 | nominal Weighted WBC | complete | [Phase 21](phases/21-nominal-weighted-wbc/PLAN.md) | current nominal simulation-only 12-DoF/42-variable Weighted WBC完成；19个10 s normal/perturbation、6个fault、solver/task/plant、fresh replay、历史回归与REVIEW/RECORD全部PASS |
| 13 | Weighted WBC ProxQP solver migration | complete | [Phase 22](phases/22-proxqp-solver-migration/PLAN.md) | 不改变Phase21 QP/WBC数学；ProxQP v0.7.3 component、oracle、deadline、19+6 formal、replay、历史回归与REVIEW/RECORD全部PASS |
| 14 | nominal acados NMPC | complete | [Phase 23](phases/23-nominal-nmpc/PLAN.md) | append-only acados v2、component、23+10 formal、fresh replay、历史回归、REVIEW/RECORD全部PASS；仅限current nominal simulation host |
| 15 | MuJoCo interactive NMPC viewer | complete | [Phase 24](phases/24-mujoco-interactive-viewer/PLAN.md) | opt-in C++ viewer复用Phase23 controller/adapter；GLFW render/headless regression/26-test suite PASS，不改变headless formal或性能口径 |
| 16 | MuJoCo mouse interaction | complete | [Phase 25](phases/25-mujoco-mouse-interaction/PLAN.md) | native camera/temporary perturb controls；build/headless/GUI smoke与26-test suite PASS，不影响formal/性能口径 |
| 17 | theory-restored wheel-aware NMPC + Minimal WBC | complete | [Phase 27](phases/27-theory-restored-minimal-wbc/RECORD.md) | 上游wheel/planner、internal interaction-wrench、16-state/acados、timing与Minimal-WBC component gates均PASS；T0～T3 formal结论为diagnosed Minimal FAIL：T0/T1/T2首失效为safety envelope，T3 `±10 mm`首失效为native NMPC stationarity audit；fault/replay/regression PASS，本Phase未add-back/retune |
| 18 | Minimal closed-loop drift / divergence attribution | complete | [Phase 28](phases/28-minimal-closed-loop-drift-attribution/RECORD.md) | T0/T1精确复现Phase27首失效并唯一归为B类NMPC净动作非恢复；WBC realization/resource与model-to-plant gates排除；T2仅得出左右不一致，不批准task或调参 |
| 19 | NMPC corrective-action root-cause audit | complete | [Phase 29](phases/29-nmpc-corrective-root-cause-audit/RECORD.md) | T0唯一关闭为terminal base-longitudinal有限域传播的P29-E；T1唯一关闭为attitude主导、wheel-rate次级的cross-state coupling P29-D；production/replay/regression PASS，未改控制律或调参 |
| 20 | NMPC corrective-action formulation repair | review | [Phase 30](phases/30-nmpc-corrective-formulation-repair/REVIEW.md) | v3判定reference已一致；20 ms wheel-rate model/state contract FAIL（P31-F），REVIEW=REWORK，production未修改 |
| 21 | Wheel-state model and measurement contract audit | review | [Phase 31](phases/31-wheel-state-model-measurement-audit/REVIEW.md) | measurement/kinematics PASS；原`P31-E/M4` dynamics归因已被Phase32的floating-base oracle与M5证据取代，production冻结 |
| 22 | Wheel-state Markov closure and constrained-dynamics derivation | review | [Phase 32](phases/32-wheel-state-markov-closure/REVIEW.md) | same-x16 pairs证明`P32-C/M5`及D/E/F hidden families；x24仅为必要增广，mesh contact hybrid阻塞smooth candidate，REVIEW=REWORK |
| 23 | Low-dimensional closure recovery via WBC manifold regulation | review | [Phase 33](phases/33-low-dimensional-closure-recovery/REVIEW.md) | Gate0/坐标/42D-104-row代数PASS；24个C1/C2 state的gain-free self authority与wrench门PASS，但cross/self `0.5126>0.5`，REVIEW=REWORK；未选gain或改production |
| 24 | 12D base NMPC + full-body WBC wheel tracking feasibility | review | [Phase 34](phases/34-base-nmpc-wheel-tracking-feasibility/PLAN.md) | REWORK：x12 model/OCP与gain-free longitudinal authority通过，但三组冻结增益的step/ramp均在tick 90--92触发WBC workspace拒绝且未达1 mm/0.2 s；按DG34-04停止，production仍为Phase27 |
| 25 | Wheel-position servo workspace failure attribution | complete | [Phase 35](phases/35-wheel-servo-workspace-attribution/RECORD.md) | P35-A：Phase27 Minimal fixed-wrench hold在tick 9起产生双轮负向spin drift，right wheel canonical delta于tick 88越过`-1 rad` live bound；Phase34六例在tick 90–92复现；production仍为Phase27 |
| 26 | Wheel-spin / rotating-mesh validity audit | complete | [Phase 36](phases/36-wheel-spin-rotating-mesh-validity-audit/RECORD.md) | P36-D：core model `2π` periodic至`3.47e-18`，但non-axisymmetric rotating collision mesh/contact manifold产生material phase response；`±1 rad`无特殊转折，live gate未修改 |
| 27 | Axisymmetric wheel collision correction + causal revalidation | review | [Phase 37](phases/37-axisymmetric-wheel-collision-causal-revalidation/REVIEW.md) | REWORK/P37-D：cylinder使contact geometry invariant且ddxi phase effect降低114.84×，但contact-on仍为off约99.54×，DG37-03 FAIL；按冻结顺序未跑Phase32/H0，production不变 |
| 28 | Wheel COM / inertia validity attribution | complete | [Phase 38](phases/38-wheel-com-inertia-validity-attribution/RECORD.md) | P38-A：compiled radial COM约0.12mm是剩余phase response的数值主因，V1多观测量降低>99.9%，V2无material作用；物理修正仍需独立axle-frame mass-property authority |
| 29 | Idealized nominal wheel model + architecture revalidation | complete | [Phase 39](phases/39-idealized-nominal-wheel-architecture-revalidation/RECORD.md) | P39-D：absolute-angle artifact关闭，但C1/C2/C3仍跨configuration/velocity/rate失败；H0为P39-F，right wheel于tick 96触发live bound；下一Phase先验证long-horizon wheel-angle safety/domain contract |
| 30 | Wheel absolute-angle domain / representation contract validation | complete | [Phase 40](phases/40-wheel-absolute-angle-domain-contract/PLAN.md) | R3 raw-unwrapped + periodic validator；P40-A+F+G；工程域±1e6 rev PASS，shadow tick111右轮失触；production gate未改、Phase34未运行 |
| 31 | Workspace contract correction + H0 production revalidation | complete | [Phase 41](phases/41-workspace-contract-correction-h0-production-revalidation/PLAN.md) | P41-A：production R3 contract生效；tick96正常继续、tick111右轮失触；shadow parity=0；未修contact loss、未运行Phase34 |
| 32 | Wheel-spin drift / contact-loss causal attribution | complete | [Phase 42](phases/42-wheel-spin-contact-loss-causal-attribution/RECORD.md) | P42-E：tick0 fixed-request非rolling equilibrium与左右contact asymmetry、后续wheel-rate-sensitive amplification均material；formal/replay、动力学闭合与zero-rate反事实PASS，未repair |
| 33 | Minimal rolling stabilization repair selection | review | [Phase 43](phases/43-minimal-rolling-stabilization-repair-selection/REVIEW.md) | REWORK/P43-U：A/B/C/D均未过tick0 native-wheel equilibrium gate，10 s nominal均在tick28–139触发rate/base/contact/WBC独立失败；未进入perturbation，不批准Phase44 tracking |
| 34 | WBC-to-plant constrained rolling realization audit | complete | [Phase 44](phases/44-wbc-to-plant-constrained-rolling-realization-audit/RECORD.md) | P44-E：addendum以逐contact/逐inequality-row regime signature和单边delta收敛修复DG44-06；396 R44-S/84 R44-P，tick0 QP→plant反号/衰减与D/native-common contact cancellation获trusted evidence，formal-v4/replay-v4及回归PASS |
| 35 | Contact-consistent rolling repair | review | [Phase 45](phases/45-contact-consistent-rolling-repair/REVIEW.md) | REWORK：compatible wrench使DG45-EQ PASS，但DG45-AUTH common projected gain在三档scale均QP/MuJoCo反号`+0.998/-1.876`；mandatory stop，未进入REAL/SHORT/10s/REAUDIT，不授权Phase46 |
| 36 | Hip-common-safe rolling realization repair | review | [Phase 46](phases/46-hip-common-safe-rolling-realization-repair/REVIEW.md) | REWORK/E：contact为unique source；native `f=D(aref-Jqacc)` oracle精确重建same-tau reaction，但等价coupled/Schur law的closed-loop diagnostic在H0 feasibility、equilibrium、branch和scale均FAIL；无可信R2 law获授权，production unchanged，下一步仅允许继续contact-response source attribution |
| 37 | MuJoCo-only workspace cleanup + interface freeze | complete | [Phase 47](phases/47-workspace-cleanup-interface-freeze/RECORD.md) | hardware source removed；ROS current WBC path、interfaces、legacy inventory 与 pre/post regression 冻结 |
| 38 | Weighted-WBC / QP realization closure | planned | Phase 48（待建） | 只研究 W_ref→W_WBC→tau→W_MJ，不改 Phase47 接口 authority |
| 39 | 12X / 16X NMPC candidate comparison | planned | Phase 49（待建） | 两 candidate 共用冻结 W_ref contract 和 current ROS/MuJoCo path |

### Retired route items

| Item | Status | Reason |
| --- | --- | --- |
| Phase 05 actuator identification | cancelled | MuJoCo-only route；见 [RETIREMENT](phases/05-actuator-torque-identification/RETIREMENT.md) |
| RobotState sensor/hardware validation | cancelled | real sensor boundary removed |
| MuJoCo–real kinematic/dynamic/contact identification | cancelled | real comparison removed |
| identified/real replay and Roll/Yaw differential identification | cancelled | hardware/identified route removed |

README 与工作流骨架属于仓库引导建设，不作为产品开发 Phase。历史 completed/REWORK
结论和 evidence 均非覆盖保留。Phase 05 已因 MuJoCo-only route reset 进入 `cancelled`，不会
恢复；Phase 46 保持 `review/REWORK`。Phase 47 只清理结构并冻结 current interface，后续
技术研究从 Phase 48 继续，旧编号不复用。

[Phase 30](phases/30-nmpc-corrective-formulation-repair/REVIEW.md) v1 direct-weight与v2 structured
cost失败证据继续非覆盖保留；v3进一步证明T0/T1的full-horizon state reference在current
equilibrium input下已经满足冻结一致性门，因此不批准继续改cost或stage feedforward。条件
Branch M在20 ms定位到wheel-center relative-rate model/state contract误差（P31-F）；这不否定
Phase28在真实state处的base acceleration direction gate，而是补充了此前未覆盖的wheel-state
rollout gate。REVIEW=`REWORK`，production仍未修改。

详细技术次序以 [MuJoCo-only 当前路线](../mujoco/mujoco-only路线.md) 为准。

## 维护规则

- 每次只修改真实发生变化的状态，不预先填写 PASS。
- `complete` 必须同时满足 REVIEW=PASS、RECORD 已写和证据链接有效。
- 阶段拆分或重排时保留原编号的历史含义，不静默复用编号。
- “顺序”表示当前执行次序，Phase 编号是稳定 ID；重排时两者可以不同。
- 发现需要改变状态、输入、模型或控制架构时，先建立技术决策任务，再继续实现。
- 模型、参数、配置和 evidence 采用追加式版本管理；新 revision/new run 不覆盖已获批 baseline，跨 revision 比较必须能同时解析两边 manifest。
