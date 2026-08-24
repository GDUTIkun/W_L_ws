# firmware/stm32

## 目录职责

STM32H7 轮腿机器人固件工程根目录，包含自研应用逻辑、硬件抽象、数学工具、FreeRTOS 与 STM32Cube 生成内容。

## 自研入口

- [`App/`](App/README.md)：任务编排、机器人应用逻辑和当前 UART2 协议实验。
- [`Hardware/`](Hardware/README.md)：电机、CAN、I2C、IMU 和延时等硬件访问层。
- [`Math/`](Math/README.md)：固件侧通用数学与 PID 工具。

`Core/`、`Drivers/`、`Middle/FreeRTOS/` 和 `MDK-ARM/` 属于生成、第三方或 IDE 相关区域，不按自研模块维护 README。

## 允许内容

- CubeMX 工程配置和可复现的源文件；
- 自研实时任务、驱动适配、状态采集、安全与控制输出；
- 与树莓派通信的候选协议实现。

## 禁止内容

- MDK 编译产物和固件二进制；
- 主机/Pi ROS2 逻辑；
- 在未冻结物理语义前把临时单位或关节映射当成正式协议。

## 上下游关系

与电机、CAN、IMU 等硬件直接交互；未来通过树莓派 Hardware Adapter 与统一 RobotState/TorqueCommand 边界连接。

## 当前状态

固件能够组织电机与状态数据，并包含 enable、e-stop、命令超时、力矩限幅和状态回传的 UART2 实验代码。其正式接口仍需独立 Phase 验证。

## 维护规则

- CubeMX 再生成后检查自研区域是否被覆盖。
- 实时循环、单位换算、限幅和安全状态的修改必须有对应验证证据。
- 不提交 `MDK-ARM` 构建输出。

