# ROS2 workspace

该 workspace 承载唯一 current 路线 `Controller Core → ROS2 → MuJoCo`。

## Build and run

```bash
colcon build --symlink-install
source install/setup.bash
ros2 launch wheel_leg_mujoco current_weighted_wbc.launch.py
```

Packages：

- `wheel_leg_core`：无 ROS/MuJoCo 依赖的 C++ Controller Core、WBC、QP 和 NMPC 资产。
- `wheel_leg_msgs`：`RobotState` 与 `TorqueCommand` ROS messages。
- `wheel_leg_ros`：消息转换和 `controller_node`。
- `wheel_leg_mujoco`：MuJoCo Adapter、`mujoco_node`、current launch 和 regression runners。

`zero_loop.launch.py` 只用于 transport smoke。Phase 34–46 历史 runner 默认不构建；
需要重放历史证据时传入 `--cmake-args -DWHEEL_LEG_BUILD_LEGACY_RUNNERS=ON`。

构建、安装和日志目录均为生成物。任何源码修改后从本目录执行 build/test。
