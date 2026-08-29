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

For deterministic experiment evidence, use the ROS-independent installed executable through the repository wrapper:

```bash
cd /home/t/W_L_ws
./.venv/bin/python tools/experiments/run_mujoco_controller_loop.py \
  --output-dir data/experiments/<new-phase16-run-id>/raw
```

That path executes one Controller tick per five MuJoCo physics steps and refuses to overwrite a non-empty output directory. The ROS launch remains a transport/schema/reset compatibility smoke, not the determinism authority.

For the opt-in fixed-base Joint PD and nominal gravity validation, use `tools/experiments/run_mujoco_joint_pd_gravity.py`. It reuses the same executable/loop and appends reference, PD, gravity, raw torque, saturation and disturbance columns; it does not establish contact, standing or hardware validity.

## Interactive nominal NMPC viewer

This is a host-side simulation viewer, not a communication endpoint or formal
performance benchmark. It reuses the Phase 23 C++ `ControllerCore` and MuJoCo
`Adapter`, starts no CSV output, and paces the simulation at its 2 ms plant time.

```bash
cd /home/t/W_L_ws
source /opt/ros/jazzy/setup.bash
source ros_ws/install/setup.bash
ros_ws/install/wheel_leg_mujoco/lib/wheel_leg_mujoco/weighted_wbc_loop \
  --model simulation/mujoco/model/scence.xml \
  --viewer true \
  --controller-mode nominal_nmpc \
  --nmpc-reference positive \
  --ticks 1000
```

The overlay shows simulation time, Core status/latch, NMPC status/age and the
separately measured Core and NMPC+WBC times. Close the window or press `Esc` to
exit. Mouse controls are native MuJoCo controls: left-drag rotates the camera,
right-drag pans it, middle-drag or wheel zooms; hold `Ctrl` while left-dragging
a selected body to apply a temporary torque, or while right-dragging to apply a
temporary force. Releasing the mouse clears that viewer-only external wrench.
Keep the default headless command with `--control-output` and `--plant-output`
for benchmark/formal data; GUI rendering, v-sync and mouse perturbations change
the wall-clock/plant behavior.
