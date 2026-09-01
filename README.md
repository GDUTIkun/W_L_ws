# W_L_ws

轮腿机器人 MuJoCo 控制仿真仓库。Phase 47 起，项目唯一正式路线是：

```text
Controller Core → ROS2 → MuJoCo
```

不再开发 STM32、串口、Hardware Adapter、树莓派部署或实物验证。Simulink baseline
仅作为冻结的算法和历史数值参考，不是当前运行链。

## Current runtime

```bash
cd ros_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch wheel_leg_mujoco current_weighted_wbc.launch.py
```

该入口固定运行 500 Hz MuJoCo、100 Hz RobotState/Weighted-WBC，并使用冻结的 H0
初始化与 nominal WBC profile。完整调用链见
[CURRENT_CONTROL_PATH](docs/mujoco/CURRENT_CONTROL_PATH.md)，接口见
[LOWER_LAYER_INTERFACE_CONTRACT](docs/interfaces/robot_state_torque_command.md)。

## Repository map

| 路径 | 职责 |
| --- | --- |
| [`ros_ws/`](ros_ws/README.md) | Controller Core、ROS messages/wrapper、MuJoCo Adapter 与 launch |
| [`simulation/`](simulation/README.md) | current MuJoCo 模型及冻结 Simulink baseline |
| [`docs/`](docs/README.md) | 技术契约、实验解释和 Phase 工作流 |
| [`tools/`](tools/README.md) | 实验、分析和维护脚本 |
| [`data/`](data/README.md) | 非 Git 大体积仿真数据与追溯清单 |

历史硬件文档保存在 [`docs/legacy/hardware/`](docs/legacy/hardware/)，不具有 current
authority。历史 Phase/evidence 保持追加式，不因路线切换改写结论。

## Engineering rules

- Simulink 不作为生产代码生成源；Controller Core 为手写 C++。
- `RobotState → Controller Core → TorqueCommand` 是唯一 ROS 控制边界。
- 模型、控制或证据结论必须由真实仿真结果支持，不能从构建成功推断。
- Phase 状态和下一步只以 [ROADMAP](docs/workflow/ROADMAP.md) 为准。
- Phase 46 保持历史 `REWORK`；Phase 47 只做路线清理和接口冻结，不改 QP/task/physics。
