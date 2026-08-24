# Baseline evidence summaries

本目录只保存小型、受控的既有结果摘要，不保存 raw MAT、完整 time series 或批量扫描。

| File | Meaning | Evidence status |
| --- | --- | --- |
| target_import_smoke_summary.csv | W_L_ws 目标路径 5 s Accelerator smoke | 本 Phase 实际运行；simulationCompleted/controlStable=true，QP/NMPC 正常 |
| flat_accelerator_smoke_summary.csv | 源快照的 5 s Accelerator 全链路 smoke | 已在源快照运行；本 Phase 将在目标路径重跑 |
| flat_1ms_start_cruise_brake_summary.csv | 1 m/s 启动—匀速—制动 | 迁入前既有通过证据 |
| flat_1ms_turning_summary.csv | 1 m/s 平地高速转向批次 | 迁入前既有证据；以 HS2_90_left_v100_yaw020 行为已验证主项 |
| flat_low_speed_360_summary.csv | 低速 360° 左转 | 物理完成，但严格 wrench/contact residual gate 未全过 |

除 target_import_smoke_summary.csv 外，其余 CSV 用于确认 baseline 身份和对照数字，不代表在 W_L_ws 路径中全部重新运行。目标路径实际执行范围和命令记录在 Phase 01 REVIEW/RECORD。
