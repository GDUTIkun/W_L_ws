# firmware/stm32/App

## 目录职责

保存 STM32 固件的机器人应用层：实时任务、状态聚合、控制命令应用和上位机通信编排。

## 允许内容

- FreeRTOS 任务入口与机器人级应用逻辑；
- 关节状态聚合、控制命令分发和安全状态机；
- 候选通信协议的编解码与统计。

## 禁止内容

- 可复用硬件驱动细节，应下沉到 `Hardware/`；
- 通用数学算法，应放入 `Math/`；
- ROS2 或 MuJoCo 依赖；
- 把文件名含 `test` 的实验协议描述为已冻结生产接口。

## 上下游关系

从 `Hardware/` 获取电机、CAN 和 IMU 数据，调用安全与控制逻辑，并向未来的树莓派 Hardware Adapter 提供状态、接收命令。

## 当前状态

`Car.cpp` 组织主要控制/数据任务；`uart_protocol_test.*` 实现当前 UART2 状态与六关节力矩命令实验链路。

## 执行器辨识实验扩展

UART2 候选协议保留原有普通命令 `0x01` 和状态 `0x81`，另增加版本为 2 的 3508+C620 电流辨识命令 `0x02` 与紧凑遥测 `0x82`。该扩展只服务 Phase 05 台架实验，不是生产协议。

辨识模式使用现有六执行器索引：

```text
0 GIM6010 left hip
1 GIM6010 left knee
2 3508+C620 left
3 GIM6010 right hip
4 GIM6010 right knee
5 3508+C620 right
```

所有多字节字段均为 little-endian。`0x02` payload 固定 24 bytes：

| Offset | Type | Field | Meaning |
| --- | --- | --- | --- |
| 0 | `u8` | version | 固定为 `1` |
| 1 | `u8` | enable | `1` 才允许所选执行器输出 |
| 2 | `u8` | estop | 非零立即使所有执行器输出归零 |
| 3 | `u8` | actuator_index | 仅允许 `2` 或 `5`（3508+C620） |
| 4 | `u8` | excitation | `0=hold`，`1=STM-local step` |
| 5 | `u8` | flags | 必须为 0；电流辨识强制关闭 C620 `Threshold_Current` |
| 6 | `u16` | reserved | 发送 0 |
| 8 | `u32` | trial_id | 新 ID 触发新试验；重复 ID 仅作为心跳，不重启阶跃 |
| 12 | `f32` | target_current_a | 目标电机电流，`A` |
| 16 | `u32` | step_delay_ms | 阶跃前零输出延迟，最大 5000 ms |
| 20 | `u32` | step_duration_ms | 阶跃持续时间，最大 2000 ms；step 模式必须非零 |

3508+C620 辨识不施加额外的保守幅值限幅；命令可覆盖 C620 协议满量程 `±20.0 A`（`±16384` 原始计数），并在该协议边界饱和。100 ms 无有效命令即立即归零；所选执行器离线进入 fault，其余五路始终强制为零。hold 使用 `0.05 A/control-cycle` 斜率限制，step 在 STM 的 1 ms 电机任务中按 STM tick 产生边沿。

`0x82` payload 固定 56 bytes，辨识模式下按 2 ms 周期发送，并暂停完整 `0x81` 状态帧：

| Offset | Type | Field | Meaning |
| --- | --- | --- | --- |
| 0 | `u8` | version | 固定为 `1` |
| 1 | `u8` | actuator_index | 当前所选执行器 |
| 2 | `u8` | actuator_type | `1=GIM6010`，`2=C620` |
| 3 | `u8` | excitation | hold/step |
| 4 | `u8` | safety_state | disabled/enabled/timeout/estop/fault |
| 5 | `u8` | flags | 实际辨识配置标志 |
| 6 | `u8` | selected_online | 所选执行器在线状态 |
| 7 | `u8` | step_state | `0=idle,1=delay,2=active,3=complete` |
| 8 | `u32` | sample_seq | 应发送采样序号；跳号表示发送忙或丢样 |
| 12 | `u32` | stm_tick_ms | STM 单调 tick，ms |
| 16 | `u32` | trial_id | 当前试验 ID |
| 20 | `f32` | current_requested_a | 未限幅的试验波形，`A` |
| 24 | `f32` | current_applied_a | 安全、限幅/斜率处理后的实际电流命令，`A` |
| 28 | `u32` | driver_command_raw | C620 低 16 位为有符号电流计数；GIM6010 为发送 float 位模式 |
| 32 | `u32` | driver_feedback_raw | C620 低 16 位为有符号反馈电流；GIM6010 为反馈力矩 float 位模式 |
| 36 | `f32` | feedback_current_a | 驱动反馈换算电流，`A` |
| 40 | `f32` | position | 当前输出轴/机构映射角度，`rad` |
| 44 | `f32` | velocity | 当前输出轴/机构映射角速度，`rad/s` |
| 48 | `u32` | command_age_ms | 距最后有效命令的 STM 时间 |
| 52 | `u32` | comm_rx_error_count | 累计接收/同步/序号/UART 错误 |

新 `trial_id` 的波形参数会被锁存；同一 ID 的后续帧只能刷新心跳及 enable/e-stop，避免重复心跳改变或重新触发阶跃。退出辨识模式需发送有效普通 `0x01` 命令帧。

## 维护规则

- 应用层不得绕过硬件层直接复制驱动实现。
- 新增通信字段时同时记录版本、单位、顺序、超时和向后兼容策略。
- 影响真机输出的改动必须包含 fail-safe、限幅和超时验证。
