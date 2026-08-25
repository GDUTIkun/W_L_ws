# wheel_leg_ros

`wheel_leg_msgs` 与 ROS 无关 `wheel_leg_core` 之间的显式转换层，以及最小 Controller wrapper。

- 订阅：`robot_state` (`wheel_leg_msgs/msg/RobotState`)
- 发布：`torque_command` (`wheel_leg_msgs/msg/TorqueCommand`)
- 参数：`max_state_age_ms`，默认 `100`

wrapper 使用主机 `steady_clock` 检查状态年龄；Adapter 必须先把采样时间映射到同一主机单调时钟域。无效、过期、未来或非单调状态被拒绝且不发布命令。
