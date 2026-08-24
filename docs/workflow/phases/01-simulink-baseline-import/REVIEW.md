# Phase 01: 迁入 Simulink 基线与验证入口 — REVIEW

Status: review

## Review Scope

- PLAN：[PLAN.md](PLAN.md)
- 审查范围：W_L_ws 当前工作树中 simulation/simulink_baseline、docs/models、Phase 01 与 ROADMAP 的新增/修改
- 审查者与日期：Codex，2026-08-24

## Implementation Check

| PLAN Task | Delivered | Evidence | Result |
| --- | --- | --- | --- |
| T01 | 来源、复制边界、冻结配置和依赖边界 | PLAN、SNAPSHOT_MANIFEST.md | PASS |
| T02 | 完整人工源码、SLX、MATLAB Project、最小 regression、full solver local runtime | simulation/simulink_baseline | PASS |
| T03 | 自包含运行 README、evidence 说明、独立模型 contract | baseline README、docs/models/simulink_mpc_wm_wbc_baseline.md | PASS |
| T04 | 目标路径 model update、接口刷新、contract tests、5 s smoke | 下表真实命令结果 | PASS |

## Validation Results

| Validation | Command / Procedure | Actual Result | Evidence |
| --- | --- | --- | --- |
| 关键复制哈希 | PowerShell Get-FileHash source/target SHA-256 | startup、WBC、NMPC dynamics/OCP、入口等完全一致 | SNAPSHOT_MANIFEST.md |
| 路径隔离 | clean MATLAB path 后 open_proformance_test(false) | root 与 WBC/controller 均解析到 W_L_ws/simulation/simulink_baseline | MATLAB stdout |
| 主模型 load/update | set_param source SimulationCommand update | exit 0；full 16-state solver available | MATLAB stdout |
| diagnostic interface contract | configure_symmetric_two_leg_simulink(true) 后查询 block | contract width=198，Coupled QP width=198，58-port demux 与 contract 一致 | MATLAB stdout；target source.slx |
| wheel dynamics contract | test_paper_wheel_relative_dynamics | PASS | MATLAB stdout |
| wheel-position coordinate contract | test_wheel_position_coordinate_contract | flat/slope left/right/common/differential error 7.7e-11 至 1.54e-10 | MATLAB stdout |
| Pfaffian contract | test_wheel_contact_pfaffian_contract | material-point error约 1e-17；dot(g)dq error=0 | MATLAB stdout |
| coupled WBC contract | test_coupled_two_leg_qp | Phase-08 QP checks passed | MATLAB stdout |
| hierarchy instrumentation contract | test_paper_hierarchical_wbc_contract | PASS | MATLAB stdout |
| 5 s Accelerator smoke | run_performance_smoke(5) with assertions | exit 0；simulationCompleted=true；controlStable=true；QP feasible=1；NMPC status/fault=0；max abs(xi_delta)=0.13135 mm | MATLAB stdout；evidence/target_import_smoke_summary.csv |
| documentation links | resolve local links in all changed docs | all changed-doc local links resolve | PowerShell check |
| source portability | search authored MATLAB source for original CodeWorkspace absolute path | none | rg check |
| generated-output boundary | audit Git ignored/untracked files | generated runtime and work outputs ignored；no raw test log remains in model source tree | git status --ignored |
| live code discovery | refresh CBM and search target entry/WBC | open_proformance_test and spatial_two_leg_qp_core indexed under W_L_ws | codebase-memory-mcp |

## Findings

### Blocking

None.

### Non-blocking

1. Optional direct 8-state common-mode solver has generated source but no top-level S-Function；startup correctly reports not built。source.slx uses the available full 16-state solver。
2. Frozen Acados MEX/DLL is Windows x64 / MATLAB R2024b specific and ignored by Git；the current local workspace is complete for replay, while a new clone must restore the controlled artifact or rebuild with external Acados/CasADi。
3. This Phase reran model update, contracts and 5 s smoke only；1 m/s straight/turning numbers are imported prior evidence, not target-path performance reruns。
4. Repository-wide Markdown audit still has a pre-existing unrelated ros_ws README link to a missing package；all documents changed by Phase 01 resolve correctly。

## Decision and Evidence Review

- 冻结决策是否被保持：控制源码、参数、plant、NMPC solver 和控制行为均保持。原 PLAN 要求 source.slx byte-identical，但真实 smoke 发现 source snapshot 的 block declaration 仍为 143，而当前 append-only diagnostics contract 已为 198。目标副本仅用项目原生 updater 刷新并保存该 interface，属于经过验证的受控偏差。
- 证据是否足以支持技术结论：是。目标路径同时有静态 load/update、五项 contract tests 和闭环 smoke，且关键源码哈希、路径隔离和输出边界均已核查。
- 是否存在需要新 Phase 的开放问题：跨主机 solver artifact/rebuild 可在后续环境固化任务处理；不阻塞当前本机基线迁入。性能和语义迁移继续按 ROADMAP Phase 02/04/11/12。

## Verdict

PASS
