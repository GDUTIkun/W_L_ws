# Phase 01: 迁入 Simulink 基线与验证入口 — RECORD

Status: complete

> 本文件在 [REVIEW.md](REVIEW.md) 结论为 PASS 后创建。

## Outcome

已将平地验证的三维 Simscape + paper Eq.(12) NMPC + weighted WM-WBC 基线完整迁入 W_L_ws，并在目标路径完成模型接口一致性刷新、结构契约测试和 5 s 闭环 smoke。

## Delivered

- [Simulink baseline](../../../../simulation/simulink_baseline/README.md)：模型、控制源码、MATLAB Project、运行入口、最小 regression 和本机 full solver runtime。
- [Snapshot manifest](../../../../simulation/simulink_baseline/SNAPSHOT_MANIFEST.md)：来源提交、复制边界、hash、runtime 与 target-only interface refresh。
- [Model technical contract](../../../models/simulink_mpc_wm_wbc_baseline.md)：坐标、状态/输入顺序、Eq.(12)、WM-WBC、slack、采样链、能力和 terrain failure 边界。
- [Evidence summaries](../../../../simulation/simulink_baseline/evidence/README.md)：平地 smoke、1 m/s 直线/转向和低速 360° 的小型既有摘要。

## Verification Evidence

- target model load/update：PASS，full 16-state/12-input NMPC S-Function available。
- diagnostic contract：source.slx Coupled QP width=198，demux 与 08-04-PAIR-HQP contract 一致。
- five contract tests：paper wheel dynamics、wheel-position coordinate、Pfaffian、coupled WBC、hierarchy instrumentation 全部 PASS。
- 5 s Accelerator smoke：simulationCompleted=true，controlStable=true，QP feasible=1，NMPC status/fault=0，max abs(xi_delta)=0.13135 mm；摘要保存在 evidence/target_import_smoke_summary.csv。
- key source hash：除已记录的 target-only SLX interface refresh 和一处 legacy absolute-path comment portability edit外，关键模型/控制源码与 source snapshot 一致。
- changed-doc links、旧绝对路径和生成物边界检查：PASS。
- target CBM index 已刷新，入口和 WBC 符号可检索。

## Decisions Confirmed

- baseline 冻结为 paper_eq12_v1 / dynamicsVersion=7、16-state/12-input NMPC、20 ms/N=20、5 ms weighted WM-WBC。
- Eq.(12) wheel-acceleration feedforward 保持开启；outer anti-split、WC-02 candidate、normal diagnostic candidates、weight/FRF/attribution/hierarchy probes 默认关闭。
- xi 仍是 wheel-center relative base-forward geometry；terrain contact basis 不重定义 xi。
- WBC slack 契约为 W* = W_mpc + slack。
- 本基线是 flat-validated、terrain-failure-characterized，不是 terrain-adapted。
- generated solver 是本机 replay asset，不是权威产品源码；权威 OCP/dynamics 由 MATLAB source 定义。

## Deviations from PLAN

- 首次 smoke 发现 source snapshot 的 SLX 输出声明为 143，而当前 WBC append-only diagnostic contract 为 198。仅在目标副本运行已有 interface updater 并保存 source.slx；未改控制参数、plant、solver 或任务行为。
- 因目标会进入版本控制，generated runtime 保留在本机但由 nested .gitignore 排除；新 clone 需要受控 artifact 或重建。

## Known Limitations and Follow-ups

- optional 8-state common-mode direct solver 当前未构建，不影响 source.slx 使用的 full solver。
- 未在本 Phase 重跑长时间 1 m/s 性能实验；对应数字保留为迁入前既有 evidence。
- frozen MEX/DLL 只验证 Windows x64 MATLAB R2024b；跨 release/OS 应重新构建。
- Phase 02 继续冻结跨 Simulink/MuJoCo/C++/real 的坐标、单位、关节顺序和接口语义。
- terrain failure 与 feasibility-aware MPC-WBC 属于后续研究，不在 baseline 迁入 Phase 内修复。

## ROADMAP Update

- 本 Phase 对应阶段：01 迁入 Simulink 基线与验证入口
- 状态变化：review → complete
- 下一建议 Phase：Phase 02 坐标系、单位、关节顺序与接口语义；Phase 05 可继续独立执行 actuator identification。

## Key Links

- [PLAN](PLAN.md)
- [REVIEW](REVIEW.md)
- [ROADMAP](../../ROADMAP.md)
