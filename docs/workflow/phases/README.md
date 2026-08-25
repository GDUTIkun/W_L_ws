# Phase 索引

本目录保存真实 Phase；各项进度以对应 PLAN/REVIEW/RECORD 和 ROADMAP 为准。

## Active

- [Phase 02：坐标系、单位、关节顺序与接口语义](02-coordinate-interface-contract/PLAN.md) — 以 Simulink 控制语义为基准冻结 canonical contract；MuJoCo 可保留方便的相对 frame，但跨边界变换必须显式且通过方向性测试。
- [Phase 05：执行器力矩辨识与模型校准](05-actuator-torque-identification/PLAN.md) — 先以 3508+C620 跑通完整链路，再覆盖 GIM6010 和 MuJoCo 参数对应验证。

## Review

None.

## Blocked

None.

## Complete

None.

## 建立新 Phase

1. 从 `../templates/` 复制 PLAN、REVIEW、RECORD 模板，但开始时只创建 PLAN。
2. 目录命名遵循 `NN-kebab-case`。
3. 在本索引和 `../ROADMAP.md` 中加入链接与状态。
4. REVIEW 进入 PASS 后才创建 RECORD，并将 Phase 移入 Complete。
