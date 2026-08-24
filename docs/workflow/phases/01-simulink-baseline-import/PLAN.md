# Phase 01: 迁入 Simulink 基线与验证入口 — PLAN

Status: `complete`

## Goal

把已经通过平地启动、直线与转向验证的轮腿机器人 Simulink 基线迁入本仓库，形成不依赖原工作区绝对路径、可由无历史记忆的新 agent 理解和启动的完整模型快照。

## Current State

- 已有：源快照 `D:\Workspace\CodeWorkspace\model\simulate\proformance_test`，包含 Simscape Multibody plant、MATLAB 控制器、`source.slx`、两层控制入口、回归脚本和已验证的 Acados solver bundle。
- 已有：源快照曾在 MATLAB R2024b 中完成路径隔离、模型 update/compile 和 5 s Accelerator smoke；平地 1 m/s 启动—匀速—制动与 1 m/s、0.2 rad/s、90° 转向另有既有实验记录。
- 缺少：目标目录中的实际源码、模型、可移植启动入口、快照清单、独立模型说明和目标路径下的结构验证。
- 证据：源快照 `SNAPSHOT_MANIFEST.md`、Research Vault 的 `projects/proformace_test/model.md` 与阶段研究报告。

## Scope

- 复制完整的人工编写模型/控制源码、`source.slx`/`source_common.slx`、MATLAB Project 元数据和最小平地回归入口。
- 复制当前默认配置实际绑定的两个 Acados solver runtime bundle，以便精确回放；排除无关历史 solver 变体、CMake 对象缓存、`slprj`、`work`、原始 MAT 日志和批量结果。
- 将所有启动和缓存路径保持为基于快照根目录的相对解析，不引用原 `CodeWorkspace`。
- 更新 `simulation/simulink_baseline/README.md`，新增独立模型说明 `docs/models/simulink_mpc_wm_wbc_baseline.md`。
- 在目标路径执行哈希、路径隔离、MATLAB 启动/模型 update 和短 smoke 验证。

## Out of Scope

- 修改、重构或调参 NMPC、WM-WBC、contact、plant 或 terrain 逻辑。
- 重新宣称 terrain adaptation 已通过；当前 terrain failure 仍是 baseline 已知限制。
- 复制历史扫描结果、完整诊断报告、图、缓存或第三方 Acados/CasADi 仓库。
- 开始 MuJoCo、ROS2 或真机接口迁移。

## Frozen Decisions

- `source.slx`、`startup.m`、`spatial_two_leg_qp_core.m` 和控制源码必须与源快照逐字节一致。
- 当前正式默认保持：16-state/12-input NMPC、20 ms 上层周期、`N=20`、5 ms WBC、Eq.(12) wheel-relative dynamics、外层 anti-split compensation 关闭。
- WC-01 修复保留；WC-02 保持 legacy default；normal task 保持 N0；hierarchy/FRF/task-attribution 诊断开关默认关闭。
- 目标仓库中的 `work/`、`slprj/`、`*.slxc` 和大体量运行结果仍为生成物，不进入基线源码边界。
- Acados solver bundle 是精确回放用的派生 runtime，不是模型权威源码；权威定义仍是 MATLAB OCP/build 源码。

## Open Questions / Decision Gates

- DG01：目标主机是否具备与已冻结 Windows MEX/DLL 兼容的 MATLAB R2024b 运行环境；若不兼容，必须使用本仓库外部提供的 Acados/CasADi 依赖重新构建，不能把旧二进制解释成跨平台产物。

## Interfaces and Compatibility

- 输入：MATLAB/Simulink 工作区参数、运动参考、状态反馈和可选外扰；SI 单位，角度内部统一为 rad。
- 输出：Simscape plant 状态、NMPC interaction wrench 请求、WM-WBC torque/feasible wrench/slack 和诊断信号。
- 必须保持：模型 block 接口、状态/输入顺序、坐标符号、solver 名称和基线默认参数不变。
- 允许改变：目标仓库 README、快照清单、Phase 文档和路径无关的顶层启动包装。

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| T01 | 固结迁入边界与来源 | 源快照、目标目录规范 | 本 PLAN、复制清单 | 人工审查排除项和 frozen decisions | done |
| T02 | 复制模型、源码和最小验证资产 | 源快照 | `simulation/simulink_baseline/` | 文件清单、大小与关键 SHA-256 对照 | done |
| T03 | 编写自包含 README 和模型说明 | 现有 model.md、源码事实 | baseline README、`docs/models` 说明 | 新 agent 所需入口/依赖/边界完整性审查 | done |
| T04 | 在目标路径验证 | 迁入快照 | 验证输出与 REVIEW | MATLAB 路径隔离、模型 update、短 smoke | done |
| T05 | 审查和归档 | T01–T04 证据 | REVIEW、RECORD、ROADMAP 更新 | REVIEW=PASS 后完成 | done |

任务状态只使用 `todo / doing / done / blocked`。

## Validation Plan

### Automated

- PowerShell SHA-256 对照：关键模型/源码 source 与 target 完全一致。
- `matlab -batch "cd('D:\Workspace\W_L_ws\simulation\simulink_baseline'); c=open_proformance_test(false); ..."`：控制器路径解析在目标快照内，`source` 可加载并 update。
- `matlab -batch "cd(...); s=run_performance_smoke(5); ..."`：仿真完成、QP 可行、无 NMPC fault。

### Manual / Evidence

- 核对 README 的入口、依赖、生成物边界、已通过/未通过能力与模型说明一致。
- 不重跑长时间性能测试；沿用源快照已有平地实验数字，并明确标成既有证据而非本次迁入重测。

## Acceptance Criteria

- [x] Scope 内交付物完成。
- [x] 自动验证通过并记录真实输出。
- [x] 必要的人工/证据验证完成。
- [x] 接口和文档与实现一致。
- [x] 开放决策得到关闭或明确转入后续 Phase。

## Execution Notes

- T02：人工源码、SLX、两组最小回归入口、full solver runtime、optional common generated source bundle 和小型 evidence 摘要已复制；关键源码哈希一致。
- T03：baseline README、snapshot manifest、evidence README 和独立模型 contract 已完成。
- T04：首次 smoke 在运动前发现 source.slx 的 Coupled QP 输出仍声明 143 维，而当前源码 contract 为 198。使用仓库已有的 append-only interface updater 仅刷新目标 SLX 的诊断端口和 demux；随后 model update、五项 contract tests 和 5 s smoke 均通过。该 target-only SLX 变化记录为相对原 PLAN 的受控偏差，不涉及控制参数或 plant。

## Blockers

None.
