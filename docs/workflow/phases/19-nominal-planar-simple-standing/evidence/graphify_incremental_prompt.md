# Graphify incremental maintenance prompt

将下面内容交给负责 Graphify 的 Claude 执行；Codex 未执行 extract/update。

```text
在 workspace /home/t/W_L_ws 中，对现有 graphify-out 做增量维护，只纳入 Phase 19 新增/修改的源码、配置与文档关系，不做全量重建或无关目录提取。

重点输入：
- docs/workflow/ROADMAP.md
- docs/workflow/phases/19-nominal-planar-simple-standing/PLAN.md
- REVIEW.md、RECORD.md、两个历史 REWORK review
- Phase 19 evidence 下的 validation.md、reuse_contract.md，以及 v3/final summary/manifest
- simulation/mujoco/config/phase19_planar_prefreeze_v3.json
- simulation/mujoco/config/phase19_planar_formal.json
- tools/experiments/run_mujoco_planar_prefreeze_v3.py
- tools/experiments/run_mujoco_planar_standing_formal.py
- ros_ws/src/wheel_leg_core 的 simple_standing 变更
- ros_ws/src/wheel_leg_mujoco/src/planar_standing_loop.cpp 与 CMake target

必须保留并连接 v1/v2 REWORK、v3 pre-freeze、formal/formal-v2 历史结果与 formal-v3 authority 的 supersedes/diagnostic 关系；不要覆盖旧节点。完成后报告实际增量输入、生成/更新节点、关系、失败项和 graph generation/version。
```
