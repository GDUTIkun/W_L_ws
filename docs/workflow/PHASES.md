# Phase 规则

## 命名

Phase 目录使用：

```text
docs/workflow/phases/NN-kebab-case/
```

- `NN` 是两位递增编号，例如 `01`、`02`。
- 名称使用简短英文 kebab-case，避免重命名造成历史链接失效。
- 已使用编号不回收；取消的 Phase 在索引中保留并说明原因。

每个 Phase 最少包含：

```text
PLAN.md
REVIEW.md
RECORD.md
```

PLAN 在执行前创建；REVIEW 在实现进入审查后创建；RECORD 只在 REVIEW 为 PASS 后创建。

## 状态流转

```text
planned → active → review → complete
             │         │
             └─────────┴→ blocked
```

- `planned → active`：PLAN 的目标、范围、任务、接口影响和验收标准完整。
- `active → review`：计划任务执行完毕或所有偏差已记录，验证证据可供检查。
- `review → active`：REVIEW 为 REWORK，需要继续修改。
- `review → complete`：仅当 REVIEW 为 PASS 且 RECORD 完成。
- 任意未完成状态可进入 `blocked`；恢复时回到原来的有效状态。

## 三类文档职责

### PLAN

回答“为什么做、做什么、按什么边界做、如何判断通过”。任务必须有稳定 ID，执行结果和偏差写在对应任务下，避免另建重复任务台账。

### REVIEW

回答“实际实现和证据是否满足 PLAN”。审查不重新设计目标；发现必须改变模型、接口或架构时，给出 REWORK，并把问题升级为技术决策。

### RECORD

回答“最终交付了什么、证据是什么、哪些决策和遗留项会影响后续”。RECORD 是后续 Phase 和 Graphify 的主要历史输入，不复制完整 PLAN 或 REVIEW。

## 任务规则

- ID 使用 `T01`、`T02`……，在 Phase 内唯一且不复用。
- 每个任务说明输入、动作、产物和验证；不能只写“完成模块”。
- 任务状态使用 `todo / doing / done / blocked`。
- `done` 只代表该任务的约定验证已执行，不代表整个 Phase PASS。
- 范围外发现写入遗留项或新 Phase，不顺带扩张当前实现。

## 技术决策与证据

- 数学模型、状态/输入定义、坐标系、物理假设、控制架构和证据解释由 Codex 冻结。
- 可以从代码确认的问题使用 CBM 解决。
- 历史设计理由、实验结论和文档关系使用 Graphify 查询。
- 需要新仿真或实验才能回答的问题必须转成任务或放行门槛，不能猜测。

## 正式实验与轻量试验

- 只有需要支撑模型、参数、接口、控制设计或 Phase 放行结论的正式设计实验，才要求 `docs/experiments/` 方法、`tools/experiments/` 执行、`data/experiments/` 数据包和 REVIEW/RECORD 证据链。
- 画图、快速 sanity check、小测试、一次性探索和不会支撑技术结论的临时仿真，不需要创建 Phase 或数据包；使用 `tools/scratch/`，输出保持本地。
- 轻量试验一旦被用作设计依据或正式结论，必须补齐为正式实验，或至少补齐其可复现输入、脚本、数据和审查证据。

## 审查门槛

REVIEW 为 PASS 必须同时满足：

1. Scope 内任务完成且无未记录偏差；
2. 自动验证和要求的人工验证有真实结果；
3. Blocking findings 为零；
4. 接口、文档与实现一致；
5. 证据依赖结论由相应证据支持。
