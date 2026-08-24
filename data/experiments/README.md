# 正式实验数据包

一个正式实验或正式仿真批次使用一个目录：

```text
data/experiments/YYYY-MM-DD-topic/
├── README.md
├── raw/          # 原始采集或原始仿真输出；Git 忽略
├── processed/    # 清洗、对齐、拟合后的数据；Git 忽略
└── figures/      # 分析图与报告；Git 忽略
```

目录名示例：`2026-08-23-actuator-torque-calibration`。

## README 最小内容

- 实验目的与关联的 `docs/experiments/` 方法；
- 关联 Phase、Git commit、模型/固件/脚本版本；
- 执行命令、设备和输入配置；
- 数据字段、单位、时间基准和关节顺序；
- raw/processed/figures 的生成关系；
- 结果摘要，以及指向 REVIEW/RECORD 的链接。

README 是 Git 跟踪的追溯清单；`raw/`、`processed/` 和 `figures/` 由 `.gitignore` 默认忽略。不要为轻量试验创建空数据包。

