# ros_ws/src

## 目录职责

保存本项目自研 ROS2 packages。包边界应围绕公共类型、控制核心、消息契约、Adapter 和 bringup 划分，而不是按主机与树莓派复制两套实现。

## 允许内容

- ROS2 package 源码、manifest、launch、config 与测试；
- 纯 C++ 公共类型和 Controller Core；
- MuJoCo 与硬件边界适配。

## 禁止内容

- colcon 的 `build/`、`install/`、`log/`；
- STM32 工程、原始实验数据或 MuJoCo 运行输出；
- 无调用方、无 Phase 决策的预留 package。

## 上下游关系

上层通过 launch/profile 组合 packages；下层分别连接 `simulation/` 的 MuJoCo 资产与 `firmware/` 的硬件协议。

## 当前状态

- `wheel_leg_bridge`：不完整的早期消息转换骨架。
- `wheel_leg_stm32_bridge`：Phase 05 实验 Hardware Adapter，复用已验证 UART 帧封装并增加力矩辨识接口；其接口只服务当前台架链路，不作为生产 schema。

除上述 Phase 05 实验切片外，后续正式 package 拆分仍必须先在接口 Phase 中确定。

## 维护规则

- 每个 package 根目录维护自己的职责、接口、依赖和验证入口。
- 消息与公共类型变更必须同步检查两个 Adapter 和 Controller。
- 不在 package README 中重复维护 ROADMAP 状态。
