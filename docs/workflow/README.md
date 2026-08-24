# 人工 Phase 工作流

本目录是项目的人工工作流入口。工作流保留 Phase、任务拆分、审查和完成记录，不依赖 GSD、自动状态机或外部执行器状态。

## 文档结构

```text
docs/workflow/
├── README.md
├── ROADMAP.md
├── PHASES.md
├── phases/
│   └── README.md
└── templates/
    ├── PLAN.md
    ├── REVIEW.md
    └── RECORD.md
```

- [ROADMAP](ROADMAP.md)：阶段总表、状态、Phase 链接和验证证据入口。
- [Phase 规则](PHASES.md)：命名、状态流转、职责和审查门槛。
- [Phase 索引](phases/README.md)：列出现有 Phase。
- [PLAN 模板](templates/PLAN.md)、[REVIEW 模板](templates/REVIEW.md)、[RECORD 模板](templates/RECORD.md)：创建新 Phase 时复制。

## 标准流程

1. 在 ROADMAP 中选择下一项，把状态从 `planned` 改为 `active`。
2. 创建 `phases/NN-name/`，复制 PLAN 模板并冻结目标、范围、任务、接口和验收标准。
3. 按任务执行；实际命令、输出和证据路径写回 PLAN 的执行记录。
4. 实现结束后把状态改为 `review`，创建 REVIEW 并独立检查范围、实现和证据。
5. REVIEW 只能给出 `PASS` 或 `REWORK`：
   - `REWORK`：回到实现，修复后重新审查；
   - `PASS`：创建最终 RECORD。
6. RECORD 完成后，更新 Phase 索引与 ROADMAP，将状态改为 `complete`。
7. 无法继续时使用 `blocked`，并在 PLAN 中记录阻塞条件和恢复条件。

## 核心规则

- README 维护稳定职责；任务状态只维护在 ROADMAP 和 Phase 文档。
- `complete` 代表验收与审查通过，不只是代码已写完。
- 涉及模型、控制、物理假设或实验解释时，必须引用真实证据。
- 开放技术决策由 Codex 负责冻结；实现不得顺带改变已冻结决策。
- 默认保持一个技术验证 Phase 为 `active`，确需并行时在 ROADMAP 明确依赖和互不冲突的边界。

## 工具使用

- CBM 用于 Phase grounding：确认 live code 的文件、符号、调用链和影响面。
- Graphify 用于读取设计历史、Phase RECORD、实验结论及工程脚本关系。
- Graphify 不能覆盖当前源码事实；历史知识与 live code 冲突时，以当前源码为准并在 PLAN 记录冲突。

## 工作流边界

项目不维护 `.planning/`、`planning_input/` 或其他 GSD 产物。历史内容通过 Git 历史追溯；当前状态只以 ROADMAP 和 Phase 文档为准。
