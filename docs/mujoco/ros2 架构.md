# ROS2 MuJoCo-only 架构

Status: `frozen — Phase 47`

```text
wheel_leg_mujoco::MuJoCoNode
       │ RobotState (SensorDataQoS, 100 Hz)
       v
wheel_leg_ros::ControllerNode
       │
       v
wheel_leg::ControllerCore → WeightedWbcController → 42D QP
       │ TorqueCommand (100 Hz)
       v
wheel_leg_mujoco::Adapter → MuJoCo ctrl → mj_step (500 Hz)
```

唯一 current launch 是 `wheel_leg_mujoco current_weighted_wbc.launch.py`。Controller Core
不依赖 ROS/MuJoCo；ROS conversions 是唯一消息重排位置；Adapter 独占 MuJoCo state、
actuator sign、source-time lag 和 receipt-time watchdog。

current launch 在 H0 发布首个 RobotState 后等待首个有效 TorqueCommand，再开始物理 step，
避免启动时的无控制漂移。simulation reset 会恢复 H0、清空 Adapter 命令历史并等待新命令；
调用方随后必须执行 `reset_controller` 以清空 Core 的 source-time epoch。

不再存在硬件分支、STM32 bridge 或第二种 production schema。`zero_loop.launch.py` 仅用于
transport smoke，direct loop 和 Phase runners 仅用于 regression/evidence。
