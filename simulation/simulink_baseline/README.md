# simulation/simulink_baseline

## 目录职责

保存已经成功复现、可作为迁移验收对照的 Simulink 仿真基线。这里的模型、参数、输入和复现入口是后续手写 C++ Controller Core 与 MuJoCo 行为对照的权威基准。

## 允许内容

- 已验证的 Simulink 模型、数据字典和依赖的脚本；
- 最小可复现输入、参数集和运行说明；
- 小型基准输出或用于一致性检查的受控数据。

## 禁止内容

- 尚未复现成功的试验模型；
- 大型仿真日志、批量结果、缓存、`slprj/` 和代码生成产物；
- MuJoCo 模型、ROS2 节点或 STM32 固件副本；
- 未经验证即标为 baseline 的模型或参数。

## 上下游关系

本目录为后续 C++ 控制算法迁移与 MuJoCo Adapter 验证提供数值和行为对照。迁移后的模块必须在适用范围内通过 Simulink/C++ 一致性检查；相关结论记录在对应 Phase 的 REVIEW 和 RECORD 中。

## 当前状态

目录由原 `reference/matlab/` 迁移而来，作为成功复现仿真的固定位置。当前工作区尚未包含 MuJoCo 工程。

## 维护规则

- 每套基线必须说明模型入口、MATLAB/Simulink 版本、依赖、运行步骤、输入和预期结果。
- 参数或模型变更前先复制为候选；只有复现和验证通过后才更新 baseline。
- 大型结果写入被忽略的输出目录，Phase RECORD 链接结论和证据位置。

