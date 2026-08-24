# wheel_leg_stm32_bridge

## 目录职责

Phase 05 使用的 ROS2→STM32 实验串口 bridge。它复用旧接口已经实机验证过的 UART 帧封装，并与当前 STM 固件的 `0x01/0x81` 普通帧、`0x02/0x82` 3508+C620 电流辨识帧保持一致。

该 package 是实验 Hardware Adapter，不冻结未来树莓派—STM32 生产协议，也不代替尚未完成的统一 RobotState/TorqueCommand 接口 Phase。

## 串口协议

所有帧均为：

```text
A5 5A type payload_len seq_lo seq_hi payload crc_lo crc_hi
```

CRC 使用 CRC16-CCITT，初值 `0xFFFF`，覆盖 `type`、`payload_len`、`seq` 和 `payload`。多字节字段为 little-endian。

支持的帧：

| Direction | Type | Payload | Meaning |
| --- | --- | --- | --- |
| ROS→STM | `0x01` | 26 bytes | 普通 enable/estop/六路力矩命令 |
| ROS→STM | `0x02` | 24 bytes | Phase 05 3508+C620 电流辨识命令 V2 |
| STM→ROS | `0x81` | 128 bytes | IMU、六执行器和安全诊断状态 |
| STM→ROS | `0x82` | 56 bytes | Phase 05 紧凑电流辨识遥测 V2 |

注意：当前 STM 的 `0x81` 在每个执行器内依次编码 `position/velocity/effort`，并非先编码六个 position、再编码六个 velocity。bridge 以当前 STM 源码为准。

## ROS 接口

节点名默认是 `stm32_bridge`，使用 private topic；默认展开如下：

| Direction | Topic | Type |
| --- | --- | --- |
| subscribe | `/stm32_bridge/normal_command` | `wheel_leg_stm32_bridge/msg/NormalCommand` |
| subscribe | `/stm32_bridge/identification_command` | `wheel_leg_stm32_bridge/msg/IdentificationCommand` |
| publish | `/stm32_bridge/normal_state` | `wheel_leg_stm32_bridge/msg/NormalState` |
| publish | `/stm32_bridge/identification_telemetry` | `wheel_leg_stm32_bridge/msg/IdentificationTelemetry` |
| publish | `/stm32_bridge/joint_states` | `sensor_msgs/msg/JointState` |
| publish | `/stm32_bridge/imu` | `sensor_msgs/msg/Imu` |
| publish | `/stm32_bridge/status` | `wheel_leg_stm32_bridge/msg/BridgeStatus` |

`normal_state.receipt_stamp` 和 `identification_telemetry.receipt_stamp` 是电脑接收时间；`stm_tick_ms` 与 `sample_seq/frame_seq` 才是 STM 同源时间/序号。两者不得混用。

## 安全行为

- bridge 只在收到合法 ROS 命令后才开始下发帧。
- ROS 命令必须持续刷新；默认超过 `100 ms` 未刷新时，bridge 发送一次 `enable=0, estop=1`，随后停止心跳，让 STM 自身的 100 ms 超时继续兜底。
- 辨识命令发生上游超时或急停后，恢复输出应使用新的 `trial_id`；复用旧 ID 不会重新触发已开始或已结束的 STM-local step。
- 非有限或超过 C620 协议满量程 `±20.0 A` 的电流、非 3508+C620 的执行器索引、非法 excitation 或超出 STM 时长约束的辨识命令会被拒绝。
- 串口写入使用非阻塞队列；未发完时不会覆盖当前帧，并通过 `status.tx_partial_writes/tx_skipped_busy` 暴露压力。
- 串口断开后每秒尝试重连，不会把断线期间积压的旧帧补发。

## 参数

默认参数见 [`config/bridge.yaml`](config/bridge.yaml)：

- `serial_device=/dev/ttyAMA4`
- `baud_rate=921600`
- `command_rate_hz=200.0`
- `command_source_timeout_ms=100`
- `frame_id=base_link`
- `joint_names=[left_hip, left_knee, left_wheel, right_hip, right_knee, right_wheel]`

## 验证状态

当前 Windows workspace 没有 `colcon`/`ros2`，因此尚未形成经过真实构建验证的运行命令。ROS2/Jazzy 环境可用后，应先验证 package 构建、协议单元测试、PTY/串口回环和板级通信，再把可复制运行入口写入本 README。

低风险 3508+C620 pilot 应先只发布 `enable=false` 的辨识命令检查回传字段，再按 Phase 05 安全清单进入小电流输出。代码完成或 topic 可见不代表硬件链路、安全动作或电流方向已经验证。
