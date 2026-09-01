# MuJoCo-only 当前路线

Status: `frozen — Phase 47`

唯一正式运行链为：

```text
MuJoCo RobotState → ROS2 controller_node → Controller Core
                  → TorqueCommand → ROS2 mujoco_node → MuJoCo
```

Simulink 只作为冻结的算法与历史数值参考，不是运行时，也不再扩展。STM32、串口、
Hardware Adapter、树莓派部署和实物验证已退出项目范围。当前运行入口和内部 authority
见 [CURRENT_CONTROL_PATH](CURRENT_CONTROL_PATH.md)，边界语义见
[LOWER_LAYER_INTERFACE_CONTRACT](../interfaces/robot_state_torque_command.md)。

后续顺序为 Phase 48 的 Weighted-WBC/QP realization closure，再到 Phase 49 的
12X/16X NMPC candidate comparison；不得从历史真机计划恢复任务。
