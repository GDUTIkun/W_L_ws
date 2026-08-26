# Graphify incremental maintenance prompt

将下面内容交给负责 Graphify 的 Claude 执行；Codex 未执行 extract/update/reflect。

```text
在 workspace /home/t/W_L_ws 中，对现有 graphify-out 做增量维护，不做全量重建，不覆盖旧节点或旧 evidence。

第一组必须补齐的历史 authority：
- docs/workflow/phases/19-nominal-planar-simple-standing/PLAN.md
- docs/workflow/phases/19-nominal-planar-simple-standing/REVIEW.md
- docs/workflow/phases/19-nominal-planar-simple-standing/RECORD.md
- docs/workflow/phases/19-nominal-planar-simple-standing/evidence/automated/2026-08-26-validation.md
- docs/workflow/phases/19-nominal-planar-simple-standing/evidence/automated/2026-08-26-formal-v4/summary.json
- docs/workflow/phases/19-nominal-planar-simple-standing/evidence/automated/2026-08-26-formal-v4/manifest.json

必须保留并连接 Phase 19 v1/v2 REWORK、v3 pre-freeze、formal/formal-v2/formal-v3演进与formal-v4最终authority之间的 supersedes/diagnostic 关系，不要把exact-planar PASS写成full-3D PASS。

第二组是新建的Phase 20 planned关系：
- docs/workflow/ROADMAP.md
- docs/workflow/phases/README.md
- docs/workflow/phases/20-nominal-3d-simple-standing/PLAN.md

Phase 20当前只有PLAN，状态为planned，没有REVIEW/RECORD/实现/evidence PASS。请建立其对Phase 18 full-3D contact plant、Phase 19 exact-planar standing、ControllerCore、RobotState/TorqueCommand、Adapter和后续WBC的依赖/范围关系；不要虚构尚未产生的配置、脚本、增益、实验或PASS节点。

完成后报告：实际增量输入、创建/更新节点、关系、未解析文件、失败项、graph generation/version，以及Phase 19 formal-v4与Phase 20 planned状态是否被正确区分。
```
