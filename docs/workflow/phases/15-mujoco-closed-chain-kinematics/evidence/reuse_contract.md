# Phase 15 Reuse and Non-Overwrite Contract

## Profile-driven 重跑

正式入口通过 `--profile`、`--config`、`--output-dir` 选择输入。runner 的数学逻辑不包含 nominal 文件路径之外的 revision 分支；未来 SolidWorks revision 或 identified plant 通过新 config/profile 提供同一 schema 的几何、模型和阈值。

每次 run 的 manifest 固定记录 profile/run ID、MuJoCo 版本、config/model/runner SHA-256、solver、seed、工作域、阈值、`supersedes` 和是否使用硬件数据。

## 非覆盖规则

- 输出目录非空时 runner 在读取/仿真前失败。
- nominal、SolidWorks revision 和 identified profile 使用不同 run ID/目录。
- 新 run 通过 manifest 的 `supersedes` 或后续 comparison artifact 建立关系，不修改旧 JSON/CSV。
- Phase 14 evidence 不作为 Phase 15 输出目录；Phase 14 回归写入新的 `data/experiments/...` 路径。

2026-08-25 的 smoke 和正式 run 结果 checks/CSV 完全一致；对正式 evidence 目录再次执行返回非零，并明确报告拒绝覆盖。

