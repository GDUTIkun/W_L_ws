# Phase 25: MuJoCo mouse interaction — RECORD

Status: `complete`

## Outcome

P24 viewer now provides native mouse camera navigation and temporary body force/torque dragging.

## Delivered

- [`weighted_wbc_loop.cpp`](../../../../ros_ws/src/wheel_leg_mujoco/src/weighted_wbc_loop.cpp)：GLFW mouse callbacks plus MuJoCo `mjvPerturb` lifecycle.
- [`wheel_leg_mujoco README`](../../../../ros_ws/src/wheel_leg_mujoco/README.md)：interactive control reference.

## Verification Evidence

- Release build, headless smoke and `DISPLAY=:0` GUI smoke PASS.
- ROS suite: 26 tests, 0 errors, 0 failures, 0 skipped.

## Decisions Confirmed

- Ctrl+left drag = temporary torque; Ctrl+right drag = temporary force; release clears it.
- No viewer interaction enters formal/performance or hardware evidence.

## Known Limitations and Follow-ups

- No persistent perturbation, parameter editor, replay or mouse automation.
- Phase 05 remains blocked by the real-hardware freeze.

## Key Links

- [PLAN](PLAN.md)
- [REVIEW](REVIEW.md)
- [ROADMAP](../../ROADMAP.md)
