# wheel_leg_ros

`wheel_leg_msgs` 与 ROS 无关 `wheel_leg_core` 之间的显式转换层，以及最小 Controller wrapper。

- 订阅：`robot_state` (`wheel_leg_msgs/msg/RobotState`)
- 发布：`torque_command` (`wheel_leg_msgs/msg/TorqueCommand`)
- 服务：`reset_controller` (`std_srvs/srv/Trigger`)

Core 只使用源系统的 `sample_time_ns` 做严格顺序和 `dt`；transport freshness 由 Adapter 使用本机 receipt steady clock 独立判断。无效或非单调状态被拒绝且不发布命令，源时间复位前必须显式调用 reset 服务。

默认 `controller.mode=zero`。Phase 17 的静态 current nominal profile 位于 `config/phase17_nominal.yaml`；其 limits 只属于 fixed-base/contact-disabled ideal-actuator 仿真，不是真实电机安全参数。
