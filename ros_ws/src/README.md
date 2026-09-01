# ROS2 packages

这里只保存 MuJoCo-only current runtime 的自研 ROS2 packages：

```text
wheel_leg_msgs
      ↓
wheel_leg_core → wheel_leg_ros → wheel_leg_mujoco
```

Controller Core 不依赖 ROS 或 MuJoCo。ROS wrapper 只负责转换和调度；MuJoCo package
拥有仿真状态、命令 watchdog、actuator mapping 和 current launch。不得在这里新增硬件
transport、第二套 RobotState/TorqueCommand schema 或替代 current runtime。
