# wheel_leg_msgs

Phase 03 冻结的聚合 ROS2 消息。字段语义以 [`docs/interfaces/robot_state_torque_command.md`](../../../docs/interfaces/robot_state_torque_command.md) 为准；ROS quaternion 使用 `[x,y,z,w]`，只在 `wheel_leg_ros` 的命名转换函数中与 Core 的 `[w,x,y,z]` 互换。
