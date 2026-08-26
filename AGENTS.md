# Project Agent Rules

本仓库用于将轮腿机器人控制系统从 Simulink 逐步迁移到 MuJoCo 和真实机器。所有工作必须遵守“先定义与验证，再提高控制复杂度”的原则。

## Source of Truth

- 仓库入口与目录职责：`README.md` 及各子目录 README。
- 阶段状态：`docs/workflow/ROADMAP.md`。
- Phase 流程与模板：`docs/workflow/PHASES.md`、`docs/workflow/templates/`。
- 当前代码事实：实际源码与 CBM 索引。
- 历史设计、实验结论和文档/脚本关系：Graphify。

本项目不使用 GSD，不创建 `.planning/`、`planning_input/` 或其他 GSD 状态和产物。

## Technical Decision Ownership

- 开放技术决策由 Codex 负责。
- 数学模型、状态/输入定义、坐标系与符号、物理假设、控制架构、辨识结构和证据解释不得仅因实现规模大而下放。
- 已冻结且没有未决技术选择的工作可以进入实现。
- 代码完成不等于模型、参数、控制效果或实验结论获批。
- 依赖证据的结论必须读取真实仿真或实验结果；没有证据时建立验证任务或 decision gate，不得猜测。

## Manual Phase Workflow

每项实质工作使用一个 `docs/workflow/phases/NN-name/` Phase：

```text
ROADMAP
  → PLAN（目标、范围、冻结决策、任务、验收和验证）
  → 实现与真实验证
  → REVIEW（PASS 或 REWORK）
  → RECORD（仅 PASS 后创建）
  → ROADMAP complete
```

- 状态固定为 `planned → active → review → complete`；无法继续时使用 `blocked`。
- PLAN 内的任务使用稳定 ID；不要另建重复任务台账。
- REVIEW 存在未解决 blocking finding 时必须为 `REWORK`。
- 范围外工作进入遗留项或新 Phase，不顺带扩张当前 Phase。
- README 只维护稳定职责与入口，进度只写入 ROADMAP 和 Phase 文档。

## Grounding

- Grounding 只把已批准设计映射到当前代码，不重新设计已冻结决策。
- 先用 CBM 查询当前文件、符号、调用关系、数据流和影响面。
- CBM 覆盖不足、文件未索引或需要查字面量时，再直接读取/搜索源码。
- Graphify 只用于历史设计理由、实验结论、Phase RECORD 和工程脚本关系。
- 当前源码与历史图冲突时，以当前源码为准，并在 PLAN 记录冲突。
- 需要新仿真或实验才能回答的问题，转成验证任务或放行门槛。

## Implementation and Verification

- 实现必须遵守当前 Phase 的 Scope、Frozen Decisions 和接口约束。
- 修改前检查工作树，保留用户已有且与当前任务无关的更改。
- 不修改第三方、生成或构建目录，除非任务明确要求。
- 自动验证记录真实命令和结果；人工或昂贵验证提供明确入口、输出位置和通过条件。
- MuJoCo PASS、Real FAIL 时停留在当前验证层排查失配，不继续增加控制复杂度。
- 最终技术验证由 Codex 负责，不能从提交、构建或任务状态推断 evidence PASS。

## Code and Knowledge Tools

### CBM

- 用于 live code discovery、调用/数据流追踪和改动影响分析。
- 新 workspace 单独索引；大规模外部修改或结构重构后刷新索引。
- 文档、第三方库和生成物不进入 CBM 主索引。

### Graphify

- 用于设计文档、实验记录、Phase 历史和工程脚本关系。
- 不替代 CBM 或源码读取。
- Codex 只允许查询本 workspace 的现有本地图，只使用 `graphify query`、`graphify path` 和 `graphify explain`。
- 在没有用户允许情况下，Codex 不得执行 Graphify 的 `extract`、`--update`、全量重建、聚类重建或语义提取，也不得为这些操作派生子代理；这些操作会消耗用户额度。
- 当现有图缺少最新内容时，Codex 应明确说明图已过期，并提供一份可直接交给其他 Claude 执行的 Graphify 增量维护 prompt；Codex 自身不得更新 `graphify-out/`。

## Automatic Subagent Delegation

主 Agent 应根据任务形态自主调用项目级子 Agent，无需等待用户逐次授权；不得为了使用子 Agent 而使用子 Agent。

- `project_scout`：只读侦察。用于非平凡的代码定位、调用链、数据流、影响面、历史设计、实验记录和 Phase 关系查询。当前代码事实走 CBM 与源码，历史关系走现有 Graphify 本地图。
- `phase_worker`：实现执行。仅当当前 Phase 的 Scope、Frozen Decisions、接口约束、文件所有权、验收条件和验证入口都已明确时使用。
- 默认最多启动一个子 Agent；只有两个任务确实独立且足够大时才并行启动两个。
- 单文件小改、强顺序依赖、仍需持续技术取舍或主 Agent 可直接快速完成的任务不委派。
- 数学模型、状态/输入定义、坐标系、物理假设、控制架构、协议语义、证据解释和最终验收始终由主 Agent 负责。
- 派发前，主 Agent 必须完成必要的 CBM 初查并传递当前 project、generation、已知符号/路径、coverage 缺口、Phase 任务 ID、范围边界、禁止修改区域和验证条件。
- 子 Agent 不是独占工作区；派发实现任务时必须明确文件或模块所有权，要求保留并适配用户及其他 Agent 的已有改动，不得回退他人修改。
- Graphify 仅允许 `graphify query`、`graphify path` 和 `graphify explain`。任何 Agent 均不得自行更新图或为更新图派生其他 Agent。
- 主 Agent 必须复核子 Agent 返回的关键证据；子 Agent 的完成、构建或测试状态不能直接解释为模型、仿真或实验 PASS。

## Directory Boundaries

- `docs/`：设计、实验方法、证据解释和人工工作流。
- `firmware/stm32/`：STM32 自研实时逻辑；HAL、FreeRTOS、MDK 等第三方/生成区域不作为普通修改面。
- `ros_ws/`：主机和树莓派共用的 ROS2 packages、launch 和配置。
- `simulation/simulink_baseline/`：成功复现、受控的 Simulink 对照基线；`simulation/mujoco/`：MuJoCo 模型与场景。
- `tools/`：非产品运行时的实验、分析和维护脚本。
- `graphify-out/`、`.codebase-memory/`、构建输出、日志和实验数据是本地生成内容，不作为产品源码。

## Current Architecture Constraints

- Simulink 是算法对照基线；Controller Core 手工迁移为 C++，不采用代码生成作为生产主线。
- 主机 profile 运行 Controller + MuJoCo Adapter。
- 树莓派 profile 运行同一 Controller + Hardware Adapter。
- MuJoCo 与真机最终使用统一 RobotState/TorqueCommand 边界。
- 树莓派—STM32 正式通信方案、消息精确 schema、关节顺序、坐标与时间语义仍需独立 Phase 冻结。
- 现有 UART2 实现是实验候选，不是已批准的生产协议。
