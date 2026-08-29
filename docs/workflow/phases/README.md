# Phase 索引

本目录保存真实 Phase；各项进度以对应 PLAN/REVIEW/RECORD 和 ROADMAP 为准。

## Active

None.

## Planned

None.

## Review

- [Phase 30：NMPC reference-consistency audit v3](30-nmpc-corrective-formulation-repair/REVIEW.md) — **REWORK**；reference本身已一致，20 ms误差定位到wheel-rate model/state contract（P31-F）；production未修改。
- [Phase 31：wheel-state model and measurement contract audit](31-wheel-state-model-measurement-audit/REVIEW.md) — **REWORK / SUPERSEDED DYNAMICS ATTRIBUTION**；measurement PASS；原M4-only结论被Phase32的floating-base M5证据取代。
- [Phase 32：wheel-state Markov closure and constrained dynamics](32-wheel-state-markov-closure/REVIEW.md) — **REWORK**；同x16证据证明`P32-C/M5`及D/E/F；x24仅是必要增广，先关闭mesh-vs-analytic contact authority。

## Blocked

- [Phase 05：执行器力矩辨识与模型校准](05-actuator-torque-identification/PLAN.md) — Phase 14 前置已 PASS；当前按用户决定冻结真机相关执行，解除冻结后从通信/计量/同步/安全 gate 恢复。

## Complete

- [Phase 01：迁入 Simulink 基线与验证入口](01-simulink-baseline-import/PLAN.md)
- [Phase 02：坐标系、单位、关节顺序与接口语义](02-coordinate-interface-contract/PLAN.md)
- [Phase 03：统一 Robot 接口与 Controller Core 骨架](03-robot-interface-controller-core/PLAN.md)
- [Phase 04：MuJoCo 基础模型与 Adapter](04-mujoco-model-adapter/PLAN.md)
- [Phase 14：MuJoCo 运动学与内部动力学验证](14-mujoco-internal-dynamics-validation/PLAN.md)
- [Phase 15：MuJoCo 完整闭链运动学与 Jacobian 验证](15-mujoco-closed-chain-kinematics/PLAN.md) — 2026-08-25 PASS；完成 210 样本被动装配、独立 FK、reduced Jacobian、有限差分、速度/虚功、方向和非覆盖复用验证，未连接真机。
- [Phase 16：Controller ↔ MuJoCo 确定性闭环运行基线](16-controller-mujoco-deterministic-loop/PLAN.md) — 2026-08-25 PASS；完成 2 ms physics、10 ms control、5-step ZOH、双时钟/reset/fail-safe、逐 tick 日志和非覆盖 replay，未新增控制算法、未连接真机。
- [Phase 17：nominal Joint PD 与重力补偿](17-nominal-joint-pd-gravity-compensation/PLAN.md) — 2026-08-25 PASS；完成解析 reduced gravity、canonical Joint PD、保持/阶跃/限幅/扰动/对称/replay 与非覆盖 profile 验证，未连接真机。
- [Phase 18：nominal 轮地接触与 floating-base plant 验证](18-mujoco-contact-floating-base-plant-validation/PLAN.md) — 2026-08-25 PASS；完成 wheel-only collision、actual-wheel normal/rolling/lateral/friction、零控制 free-flight/touchdown、base state/reset/replay 和历史回归，未连接真机、未做站立。
- [Phase 19：exact 2D sagittal 简单站立](19-nominal-planar-simple-standing/PLAN.md) — 2026-08-26 PASS；formal-v4 完成 11 个 10 s normal/perturbation、4 个 fault cases、fresh replay 与历史回归；仅限 current nominal exact-planar simulation。
- [Phase 20：nominal 完整 3D 简单站立](20-nominal-3d-simple-standing/PLAN.md) — 2026-08-26 PASS；formal-v3完成19个10 s normal/perturbation、6个fault cases、plant/contact/slip/closure、fresh replay与历史回归；仅限current nominal full-3D simulation。
- [Phase 21：nominal Weighted WBC](21-nominal-weighted-wbc/PLAN.md) — 2026-08-28 PASS；formal-v1完成12-DoF/42-variable Weighted WBC、19个10 s normal/perturbation、6个fault、solver/task/plant、fresh replay与历史回归；仅限current nominal full-3D simulation。
- [Phase 22：ProxQP solver migration](22-proxqp-solver-migration/PLAN.md) — 2026-08-28 PASS；保持Phase 21冻结的42D/104-row Weighted WBC与canonical接口，完成ProxQP v0.7.3 component/oracle、19+6 formal-v2、fresh replay、历史回归与非覆盖审计；仅限current nominal simulation host。
- [Phase 23：nominal acados NMPC](23-nominal-nmpc/PLAN.md) — 2026-08-29 PASS；append-only acados v2、23+10 formal、fresh replay和兼容性回归完成；仅限current nominal simulation host。
- [Phase 24：MuJoCo interactive NMPC viewer](24-mujoco-interactive-viewer/PLAN.md) — 2026-08-29 PASS；opt-in GLFW viewer复用Phase23 C++ controller/adapter，headless formal和性能口径不变。
- [Phase 25：MuJoCo mouse interaction](25-mujoco-mouse-interaction/PLAN.md) — 2026-08-29 PASS；native camera与temporary force/torque dragging，仅限viewer。
- [Phase 27：theory-restored wheel-aware NMPC + Minimal WBC](27-theory-restored-minimal-wbc/RECORD.md) — 2026-08-29 PASS；上游物理/component gate与fault/replay/regression PASS，T0～T2首失效为safety envelope、T3 `±10 mm`首失效为native NMPC stationarity，结论为diagnosed Minimal FAIL且未add-back/retune。
- [Phase 28：Minimal closed-loop drift / divergence attribution](28-minimal-closed-loop-drift-attribution/RECORD.md) — 2026-08-29 PASS；T0/T1唯一归为B类NMPC净动作非恢复，WBC realization/resource与model-to-plant gates排除；T2左右不一致且不作primary attribution，未批准task或调参。
- [Phase 29：NMPC corrective-action root-cause audit](29-nmpc-corrective-root-cause-audit/RECORD.md) — 2026-08-29 PASS；T0唯一归为terminal base-longitudinal有限域传播P29-E，T1唯一归为attitude主导、wheel-rate次级的cross-state coupling P29-D；offline-only诊断未改变production控制律或调参。

## 建立新 Phase

1. 从 `../templates/` 复制 PLAN、REVIEW、RECORD 模板，但开始时只创建 PLAN。
2. 目录命名遵循 `NN-kebab-case`。
3. 在本索引和 `../ROADMAP.md` 中加入链接与状态。
4. REVIEW 进入 PASS 后才创建 RECORD，并将 Phase 移入 Complete。
