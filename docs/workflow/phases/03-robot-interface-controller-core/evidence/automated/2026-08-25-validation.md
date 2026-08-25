# Phase 03 Automated Validation — 2026-08-25

## Environment

- OS/toolchain: Ubuntu 24.04, CMake 3.28.3, GNU C++ 13.3.0
- ROS: Jazzy (`/opt/ros/jazzy`), `colcon` available
- CBM post-implementation generation: `2026-08-25T03:52:06Z`, 3825 nodes / 6385 edges

## Pure C++ Core

Command:

```bash
cmake -S ros_ws/src/wheel_leg_core -B build/phase03-wheel-leg-core \
  -DBUILD_TESTING=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build/phase03-wheel-leg-core --parallel
ctest --test-dir build/phase03-wheel-leg-core --output-on-failure
```

Actual result: configure/build succeeded with `-Wall -Wextra -Wpedantic -Werror`; `wheel_leg_core_contract` passed, 1/1 tests, 0 failures. The test keeps assertions active under `NDEBUG` so the RelWithDebInfo run is meaningful.

## ROS2 Workspace

Command:

```bash
cd ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
colcon test
colcon test-result --verbose
```

Actual result: 4 packages built (`wheel_leg_core`, `wheel_leg_msgs`, `wheel_leg_ros`, `wheel_leg_stm32_bridge`). Final test summary: 11 tests, 0 errors, 0 failures, 0 skipped.

Covered behavior:

- finite values, quaternion norm and `q/-q`, contact enum, fixed joint order;
- future/stale/non-monotonic time, reset, derived `dt`, deterministic zero output;
- canonical FLU ↔ legacy `[forward,right,up]` translational-field fixture;
- C++ ↔ ROS round-trip and `[w,x,y,z] ↔ [x,y,z,w]` quaternion reorder;
- valid pub/sub state produces six zero torques; stale and NaN samples produce no command.

The full build emitted clock-skew warnings only for pre-existing Phase 05 `wheel_leg_stm32_bridge` files whose mtimes are about seven hours ahead of the environment clock. That package still built and its four protocol tests passed. No Phase 03 source emitted the warning.

## Dependency Boundary

Command:

```bash
rg -n '#include.*(rclcpp|mujoco|serial|can)|find_package\((rclcpp|geometry_msgs|wheel_leg_msgs|mujoco)' \
  ros_ws/src/wheel_leg_core/include ros_ws/src/wheel_leg_core/src \
  ros_ws/src/wheel_leg_core/CMakeLists.txt
```

Actual result: no match; boundary check PASS. `wheel_leg_core` uses only C++17 in its compiled interface/library. `ament_cmake` is optional packaging support, not a compiled runtime dependency.

## Intermediate Failure Retained

The first ROS build failed in `wheel_leg_ros` because the exported Core target was linked as bare `-lwheel_leg_core`. Inspection of the generated export showed the authoritative imported target `wheel_leg_core::wheel_leg_core`; the downstream CMake was corrected and the full workspace build then passed. The first pub/sub assertion also counted a queued prior valid sample; the test now drains the executor before asserting rejection behavior. The original generated failure log was under `ros_ws/build/wheel_leg_ros/Testing/Temporary/LastTest.log` and was superseded by the passing final run.

## CBM Verification

Post-implementation CBM search found `ControllerCore`, `ControllerNode`, `validateRobotState`, conversion functions and the ROS main entry point. LSP evidence traces `ControllerNode::onState` to `ControllerCore::step`, and `step` to `validateRobotState`. Exact paths and scopes under the three new packages report `no_recorded_issue`; this remains a best-effort coverage signal, with compiled source and tests as ground truth.
