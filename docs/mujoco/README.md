# docs/mujoco

## 目录职责

保存轮腿机器人从 Simulink 迁移到 MuJoCo 和真机过程中，关于物理语义、执行器、传感器、动力学、ROS2 边界和实验验证的技术文档。

## 主要入口

- [Simulink → MuJoCo → Real 流程](simulink%202%20mujoco%202%20real流程.md)：总验证链路与逐层放行原则。
- [ROS2 架构](ros2%20架构.md)：Controller、RobotState、TorqueCommand 和 Adapter 的目标边界。
- [拆机力矩测试](拆机力矩测试.md)：执行器力矩映射、摩擦与惯量实验。
- [传感器低通](传感器过低通.md)：装机后传感器滤波判断。
- [等效转动惯量测量](等效转动惯量测量.md)：真实关节等效惯量的测量与解释。

## 允许内容

- 已批准的物理、控制和接口设计；
- 实验方案、测量口径、验收门槛和证据解释；
- MuJoCo 与真机的差异、失败模式和纠偏结论。

## 禁止内容

- MuJoCo 模型、ROS 节点或 STM32 源码；
- 大型原始采样数据和自动生成报告；
- 尚未验证却标记为 PASS 的结论。

## 上下游关系

本目录给 `simulation/`、`ros_ws/` 和 `firmware/` 提供技术约束；相关实现、测试和实验结果应由 Phase RECORD 链接回来。

## 当前状态

文档定义了目标迁移路线，但 MuJoCo 工程、统一 ROS2 接口及控制核心尚未完整实现。

## 维护规则

- 新实验结论必须附真实数据或 Phase RECORD 链接。
- 坐标系、单位、状态定义和接口语义的变更必须显式记录。
- 项目进度放入 `docs/workflow/ROADMAP.md`，不在本目录重复维护。

