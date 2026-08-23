
# Phase: <Phase 名称>

## 1. Goal

一句话说明这个 Phase 最终要实现什么。

例如：

> 完成 `<功能/模块>`，使其能够 `<最终可观察结果>`。

---

## 2. Current State

说明当前已经有什么、现状是什么、为什么需要做这个 Phase。

- 当前已有：

  - `<已有功能/模块>`
  - `<已有数据/脚本/接口>`
- 当前存在的问题：

  - `<问题 1>`
  - `<问题 2>`
- 已有的重要结论：

  - `<已经验证或冻结的结论 1>`
  - `<已经验证或冻结的结论 2>`
- 本 Phase 的主要动机：

  - `<为什么现在要处理>`

对于已经通过历史工程或实验得到的结论，只保留与本 Phase 有关的最终结论，不必重复全部历史过程。

---

## 3. Scope

本 Phase **需要完成**：

- `<工作范围 1>`
- `<工作范围 2>`
- `<工作范围 3>`
- `<工作范围 4>`

这里描述的是本 Phase 需要解决的**技术与工程问题边界**。

不要求 Chat 精确确定最终代码文件、函数或调用关系，这些内容由后续 Codex Grounding 根据真实代码库确认。

---

## 4. Out of Scope

本 Phase **明确不做**：

- `<不处理的内容 1>`
- `<不处理的内容 2>`
- `<留给后续 Phase 的内容>`
- `<禁止顺带重构/扩展的内容>`

如果某项工作需要明显扩大当前 Phase 的技术范围，应保留到后续 Phase，而不是在实现过程中自行扩展。

---

## 5. Implementation Decisions / Constraints

记录 Chat 阶段已经确定的技术决策、限制和偏好。

### 已批准技术决策

- 使用：`<语言 / 框架 / 工具>`
- 模型 / 状态 / 输入定义：
  - `<已确定内容>`
- 坐标系 / 符号约定：
  - `<已确定内容>`
- 数据口径：
  - `<command / realized / measurement 等>`
- 算法 / 模型结构：
  - `<已冻结结构>`
- 其他：
  - `<决策 1>`
  - `<决策 2>`

### 工程约束

- 优先复用：`<已有模块 / 接口 / 数据>`
- 必须保持：`<兼容性 / 接口 / 文件结构>`
- 不允许：`<破坏性修改 / 某类方案>`
- 异常处理要求：`<fail-closed / fail-safe / 单 case 失败继续批处理等>`
- 性能或资源限制：`<如有>`

> 未明确规定的**代码实现细节**由后续 Codex Grounding 根据当前真实代码结构确认。

> 如果尚未确定的问题涉及数学模型、状态定义、输入定义、物理假设、控制架构、辨识结构或其他技术规格，不得由 Grounding、GSD Planner 或 CC Executor 擅自决定，应保留为后续 `CODEX_DECISION`。

---

## 6. Expected Changes

Chat 阶段根据当前已知信息，给出可能涉及的实现区域。

预计可能涉及：

- `<path/to/file1>`
- `<path/to/file2>`
- `<path/to/module/>`
- `<Simulink / Simscape model>`

可能需要读取或参考：

- `<related/file1>`
- `<related/file2>`
- `<已有 runner / validation / analysis infrastructure>`

可能产生：

- `<输出文件>`
- `<日志>`
- `<分析结果>`
- `<测试结果>`
- `<batch runner>`
- `<工程记录>`

> 以上仅作为 Chat 阶段提供给 Codex Grounding 的**实现区域线索**。

> 最终实际文件、symbol、function、subsystem、caller/callee、dependency path 和 impact surface，由 Codex Grounding 使用 CBM 根据当前代码库确认。

> Chat 不需要为了猜测具体代码结构而扩大设计文档。

---

## 7. Acceptance Criteria

Phase 完成必须满足：

### Implementation Completion

- [ ] `<主要功能 / 模型 / 模块已经实现>`
- [ ] `<输入输出接口正确>`
- [ ] `<正常情况能够正确工作>`
- [ ] `<异常情况能够正确处理>`
- [ ] `<必要 logging / runner / analysis 工具已经建立>`
- [ ] `<已有相关功能无明显回归>`
- [ ] `<自动 smoke / regression 验证通过>`

### Evidence / Technical Approval

如果本 Phase 包含模型、算法或物理假设验证：

- [ ] `<关键实验完成>`
- [ ] `<关键验证指标达到要求>`
- [ ] `<cross-condition / cross-seed / cross-yaw 等通过>`
- [ ] `<长期 / multi-step / free-run 等达到要求>`
- [ ] `<残差 / PSD / 稳定性等证据得到检查>`
- [ ] `<开放技术问题得到足够证据支持>`
- [ ] `<由 Codex 完成最终技术判断>`

最终结果必须满足本 Phase Goal。

不得仅因为代码实现完成，就自动认为 evidence-dependent 的技术结论已经成立。

---

## 8. Verification

### 8.1 Agent-Automated Validation

执行 Agent 可以自动完成的验证包括：

- `<静态检查>`
- `<MATLAB 短测试>`
- `<单元测试>`
- `<small regression>`
- `<smoke simulation>`
- `<logging sanity check>`
- `<小幅正负 excitation>`
- `<其他耗时可控的验证>`

需要记录：

- 执行命令；
- 测试结果；
- 失败项及原因；
- 生成的关键产物。

---

### 8.2 Manual / Expensive Validation

如果完整验证包含长时间仿真、大量 case 或高成本实验：

Agent 不应自动长时间运行全部验证。

需要准备明确的人工入口，例如：

```text
runner / MATLAB command
case 数量
参数范围
输出路径
expected outputs
```

由用户手动运行：

- `<full batch>`
- `<cross-seed>`
- `<cross-yaw>`
- `<长时间 free-run>`
- `<大规模 parameter sweep>`
- `<其他高成本实验>`

完整数据产生后，再进入后续分析或技术判断。

---

### 8.3 Evidence Interpretation

如果最终结论涉及：

- 模型结构是否正确；
- 某状态是否应该进入模型；
- 某输入参数化是否合理；
- 某 residual 的物理来源；
- 是否需要改变算法 / 控制 / 辨识结构；
- 是否批准进入下一阶段；

则：

```text
实验 / 仿真生成证据
        ↓
CODEX_DECISION
        ↓
技术解释 / 接受 / 否决 / 下一步方向
```

实现 Executor 不得根据实验现象自行修改已经冻结的技术规格。

如果 CC 发现必须改变冻结的模型、状态、输入或架构才能继续，应停止并报告，而不是自行重新设计。

---

## 9. Notes / Open Questions

### 9.1 Notes

补充当前 Phase 的重要背景：

- `<补充背景>`
- `<Chat 阶段已经确定的信息>`
- `<需要后续 Grounding 特别注意的历史结论>`
- `<已知失败路线 / 禁止重复方案>`
- `<与其他 Phase / 工程的关系>`

只保留与当前 Phase 有实际影响的信息。

---

### 9.2 Codebase Questions

这些问题 Chat 当前无法确认，但**可以通过当前代码库确认**。

例如：

- `<这个信号实际在哪里生成？>`
- `<当前 WBC mapping 由哪个 symbol 实现？>`
- `<哪个函数负责 contact mapping？>`
- `<是否已经存在可复用 runner？>`
- `<当前 logging 是否已经包含某字段？>`
- `<某函数的 caller/callee 是什么？>`

这些问题交给：

```text
Codex Grounding
+
CBM
```

解决。

如果涉及历史设计理由或过去工程结论，可由 Grounding 按需使用 Graphify。

---

### 9.3 Technical Decision Questions

这些问题仍需要开放式技术判断。

例如：

- `<最终状态向量应该是什么？>`
- `<某物理量应该作为 state / input / scheduling variable 还是删除？>`
- `<最终 input parameterization 应采用什么结构？>`
- `<是否需要改变控制架构？>`
- `<应该采用哪种模型结构？>`

这些问题后续必须形成：

```text
CODEX_DECISION
```

不得由 CC Executor 自行决定。

如果没有：

```text
None.
```

---

### 9.4 Evidence-Dependent Questions

这些问题不能仅靠当前代码或推理回答，必须依赖新的仿真、实验或数据。

例如：

- `<某 residual 的物理来源是什么？>`
- `<候选模型能否通过 cross-yaw？>`
- `<contact deformation 是否能够解释当前 drift？>`
- `<某参数是否能够跨工况保持稳定？>`

处理流程应为：

```text
实现 / 仿真
    ↓
产生证据
    ↓
CODEX_DECISION
    ↓
最终技术结论
```

不得在证据产生之前预设结论。

如果没有：

```text
None.
```

---

## Design Completion Principle

这份文档负责回答：

```text
这个 Phase 为什么做？
需要解决什么？
当前技术上已经确定什么？
还有哪些技术问题没有确定？
最终需要什么证据才能认为它完成？
```

这份文档**不负责**：

- 精确确认当前代码 symbol / caller / callee；
- 完整分析代码 impact surface；
- 决定 CC / Codex 执行路由；
- 决定 Flash High / Flash Max；
- 拆分 GSD PLAN；
- 实现代码。

后续流程为：

```text
Chat Design
    ↓
design.md
    ↓
Codex Grounding
├─ Graphify：历史工程 / 设计理由
├─ CBM：当前代码 / 调用关系
└─ Execution Routing
    ↓
grounding.md
    ↓
GSD Planner Lite
    ↓
PLAN
    ↓
CODEX / CC mixed execution
```
