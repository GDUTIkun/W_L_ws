# Project Agent Workflow

> 将本模板合并到目标项目 `AGENTS.md`，并根据真实项目补充目录边界、技术栈和验证命令。删除所有不适用内容。

## Technical Decision Ownership

- 开放的技术决策属于 Codex。
- 数学模型、状态/输入定义、物理或业务假设、控制/系统架构、识别结构和 evidence 解释不得仅因实现规模大而下放。
- 已冻结且没有未决技术选择的实现可以交给执行器。
- 代码完成不等于技术结论或 evidence 获批；Codex 负责最终技术验证。

## Planning Pipeline

```text
Chat（技术推理/设计）
  -> planning_input/design/<Phase>--design.md
  -> Codex Grounding（CBM 当前代码结构；可选 Graphify 历史知识）
  -> planning_input/grounding/<Phase>--grounding.md
  -> GSD Planner
  -> .planning/phases/<Phase>/NN-PLAN.md
  -> 执行器实施与生成 evidence
  -> Codex Verifier
```

计划输入与 GSD 自动产物分离：

```text
planning_input/design/       强技术设计，不写具体代码变更清单
planning_input/grounding/    已落地到当前代码结构的文件/符号/影响面规格
.planning/                   GSD 自动生成的 PROJECT/ROADMAP/STATE/PLAN/SUMMARY 等
```

## Grounding

- Grounding 阶段不重新设计已批准的技术决策。
- 使用 codebase-memory-mcp 查询当前代码结构、符号、调用关系和改动面。
- Graphify 仅用于已有研究知识、历史设计理由和实验结论。
- 当前代码与历史知识冲突时，以当前代码结构为准，并记录冲突。
- 未解决的技术决策转成明确的 `CODEX_DECISION` 任务，不伪装为冻结实现。

## Execution Routing

```text
CODEX_DECISION = 存在开放技术决策
Cross-AI/Executor = 规格已冻结的实现
```

- 难度不是保留或下放工作的唯一依据；关键是规格是否冻结。
- 执行器必须严格按 PLAN 工作，不得扩张范围或重做架构决策。
- evidence 依赖型结论必须读取真实 evidence，不得从代码完成推断成功。

## Code and Knowledge Tools

### codebase-memory-mcp

- 用于当前代码的函数、类、模块、调用关系、数据流和改动影响分析。
- 新 workspace 必须单独索引；不要迁移其他机器的 MCP 索引。
- 大规模重构后重新索引。

### Graphify（可选）

- 只查询目标项目真实存在的知识图。
- 用于论文、笔记、实验、历史设计理由和知识关系。
- 不用于替代 live code discovery。
- 如果目标项目没有知识图，删除或禁用本节依赖。

## Long-Running Executor Wait Policy

- Orchestrator 只协调，不频繁轮询健康 executor。
- executor 启动后，停止与该 PLAN 冲突的实现、测试和工作树修改，等待原生完成结果。
- Cross-AI 使用 `.planning/config.json` 中的同步命令，并等待结构化结果。
- 长运行本身不等于 stall；文本沉默、日志量或 token 使用量不是健康指标。
- stall surveillance 只能作为异常恢复后备，不能变成持续轮询。
- completion 后再检查 SUMMARY、提交和验证证据。

## Executor Watchdog Event Protocol

- 生命周期事件通过 `.planning/scripts/gsd-event.ps1` 发布。
- 每次 PLAN 执行生成唯一 `execution_id`，同一次执行始终复用该值。
- wrapper 发布 `STARTED` 和终止事件；执行器只在自然边界发布 `PROGRESS` 或长操作前发布 `LONG_OPERATION`。
- 不发布周期心跳。
- 成功必须有真实验证、最终提交和对应 SUMMARY；失败必须有简洁原因。
- 同一 PLAN 必须使用项目级 OS 文件锁拒绝并发执行。
- watchdog 只能观察并写通知；不得杀死、重启、恢复或接管 executor。

## Project Directory Boundaries

- Agent_Config/ is the workflow migration reference; do not put project implementation or generated workflow state inside it.
- planning_input/design/ contains approved technical designs; planning_input/grounding/ contains codebase-grounded change specifications.
- .planning/ contains GSD configuration, executor scripts, and GSD-generated project/phase artifacts; .planning/runtime/ and local Cross-AI settings are untracked.
- .gsd/ is GSD runtime state. No product-source, test, data, or experiment directories exist yet; establish their boundaries when the project is initialized.
- Graphify is disabled until this workspace has a real, local knowledge graph.
