# wheel_leg_mujoco

MuJoCo 3.7.0 Adapter for the Phase 03 canonical `RobotState` / `TorqueCommand` boundary.

- Physics step: `0.002 s` (500 Hz), independent of ROS callbacks.
- State publish: every 5 physics steps by default (100 Hz).
- Joint mapping: `q_C=-q_M+b`, `dq_C=-dq_M`, `tau_M=-tau_C`.
- Base state: torso `base_control_frame` pose and Jacobian-derived twist in world FLU.
- Contact: named left/right wheel collision geom against `floor` only.
- Safety: command path defaults disabled; invalid, late, timed-out or reset-old commands write zero.

Run after building the workspace:

```bash
cd ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to wheel_leg_mujoco
source install/setup.bash
ros2 launch wheel_leg_mujoco zero_loop.launch.py floating_base:=false
```

Use `floating_base:=true` only for bounded free-fall/state/reset sanity. The launch loads `config/fixed.yaml` by default; an alternative parameter file can be supplied with `config_file:=...`, while the explicit `floating_base` launch argument has final precedence.

Reset is explicit and ordered:

```bash
ros2 service call /reset_simulation std_srvs/srv/Trigger '{}'
ros2 service call /reset_controller std_srvs/srv/Trigger '{}'
```

Simulation resets first so its command/history and ctrl are immediately zero; Controller resets second so no old-epoch state can be accepted between the two calls.

The fixed mode is a mapping/zero-loop smoke. Floating mode is only a bounded state/reset sanity check; neither mode demonstrates standing or calibrated dynamics.
