# ros_ws

## 目录职责

主机和树莓派共用的 ROS2 workspace。未来承载公共消息、纯 C++ Controller Core、ROS wrapper、MuJoCo Adapter、Hardware Adapter 以及两套部署 profile。

## 目标部署

- 主机：Controller + MuJoCo Adapter。
- 树莓派：同一 Controller + Hardware Adapter。
- 两端使用统一的聚合 RobotState/TorqueCommand 边界。

## 允许内容

- ROS2 packages、launch、配置和 ROS 边界测试；
- 与 ROS 解耦的公共 C++ 类型和控制核心；
- MuJoCo/硬件 Adapter 的 ROS 接口层。

## 禁止内容

- STM32 固件或第三方 MuJoCo 源码；
- `build/`、`install/`、`log/` 和 rosbag 原始数据；
- 在专门接口 Phase 之前擅自冻结消息 schema、关节顺序或树莓派—STM32 传输方案。

## 当前状态

- [`src/wheel_leg_bridge/`](src/wheel_leg_bridge/README.md) 是早期消息转换骨架，依赖尚未放入仓库的 `wheel_leg_common` 和 `wheel_leg_msgs`，当前不能独立构建。
- [`src/wheel_leg_stm32_bridge/`](src/wheel_leg_stm32_bridge/README.md) 是 Phase 05 的自包含实验串口 bridge，覆盖当前 STM 普通帧和辨识帧；它不冻结未来统一 RobotState/TorqueCommand 或生产通信协议。

## 维护规则

- 只有真实构建通过后才在本 README 添加构建命令。
- Controller Core 不直接依赖 MuJoCo API 或串口/CAN 传输。
- 主机与树莓派 profile 必须复用同一控制核心和接口语义。
