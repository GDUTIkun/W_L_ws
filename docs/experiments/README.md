# docs/experiments

## 目录职责

保存**正式设计实验**的稳定方法和验收标准。正式实验是指结果将影响模型、参数、接口、控制设计或 Phase 放行结论的实验。

## 适用内容

- 执行器静态力矩标定、摩擦、惯量、传感器滤波、动力学和接触验证方法；
- 激励设计、测量口径、设备配置、安全措施和通过条件；
- 可被多次复用的实验规程。

## 不适用内容

- 单次运行的原始数据、图和日志；放在 `data/experiments/`。
- 实验执行脚本和分析代码；放在 `tools/experiments/`、`tools/analysis/`。
- 临时画图、快速 sanity check、小测试和一次性探索；它们不需要进入本目录。

## 正式实验最小链路

```text
本目录的实验方法
  → tools/experiments/执行或采集脚本
  → data/experiments/<run-id>/数据与 README
  → tools/analysis/分析脚本与图
  → Phase REVIEW / RECORD 的结论
```

只有需要复现、审查或支撑技术结论时才走这条链路。轻量试验遵循 `tools/scratch/README.md`。

## 维护规则

- 实验方法写清对象、前置条件、输入、记录字段、输出、风险控制和通过标准。
- 一项方法可以被多次 run 复用；不要把一次实验结果写回方法文档。
- 若结果推翻既有模型或设计，建立 decision gate 或新 Phase，不静默修改结论。

