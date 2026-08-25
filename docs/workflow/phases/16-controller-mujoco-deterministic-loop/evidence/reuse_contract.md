# Phase 16 复用与非覆盖契约

- 固定循环实现只读取 model path、episode/tick 数和 physics-steps-per-control；Controller/Adapter 边界不因 plant profile 改变。
- Python wrapper 从 versioned config 取得 scene、timing、fault schedule 和阈值，并为 config、scene、base model、Controller、Adapter、runner source、wrapper 和 executable 计算 SHA-256。
- 每次运行必须使用空的新目录；C++ 拒绝覆盖已存在 CSV，wrapper 拒绝非空 output directory。
- `nominal_a.csv`、`nominal_b.csv`、`faults.csv` 都保留完整 control-tick 行，不能只保存汇总；新 schema 需要提高 `schema_version`。
- SolidWorks revision、identified plant 或新 Controller build 必须创建新 config/new run，保留旧 manifest 和输出。新结果可以比较旧结果，但不得改写 Phase 16 nominal evidence 或宣称继承其数值 PASS。
- Phase 14/15 runner 和 evidence 是独立入口。本 Phase 回归写入新的 `data/experiments/2026-08-25-phase16-*-regression/` 目录，没有覆盖历史正式结果。

