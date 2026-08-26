# Phase 21: 6010 CAN Position Control — PLAN

Status: `blocked`

## Goal

Expose native ODrive CAN Simple position and velocity control for each GIM6010 node without changing the existing torque-command behavior.

## Scope

- Add `Set_Target_Position()` (revolutions) and `Set_Target_Angle()` (native axis radians) to `Class_Motor_DJI_GIM6010`.
- Add `Set_Target_Velocity()` (revolutions per second) to `Class_Motor_DJI_GIM6010`.
- Encode `Set_Controller_Mode(3,3)` and `Set_Input_Pos` (`0x00c`) exactly as specified by `firmware/stm32/reference/6010_can.md`.
- Encode `Set_Controller_Mode(2,2)` and `Set_Input_Vel` (`0x00d`) exactly as specified by `firmware/stm32/reference/6010_can.md`.
- Make the existing CAN scheduler use the command ID selected by the motor's active control method.

## Out of Scope

- Defining a leg-frame, joint-frame, gearbox, or multi-turn position convention.
- Adding a UART/ROS command for position control.
- Running or enabling real-machine tests while the project-wide hardware freeze is in effect.

## Frozen Decisions

- CAN identifier: `(node_id << 5) | command_id`, standard 11-bit frame, little-endian payload.
- Position mode uses control/input mode `3/3`; velocity mode uses `2/2`; torque mode remains `1/1`.
- `Set_Target_Position()` accepts the device-native `Input_Pos` unit, revolutions. `Set_Target_Angle()` is only a native-axis-radian convenience conversion and must not be confused with `Get_Now_Angle()`'s mapped leg frame.

## Tasks

| ID | Task | Input | Deliverable | Validation | Status |
| --- | --- | --- | --- | --- | --- |
| T01 | Add native position/velocity APIs and packet encoders | 6010 CAN reference | Driver APIs and 8-byte `Set_Input_Pos` / `Set_Input_Vel` payloads | Source inspection and encoding check | done |
| T02 | Select command CAN ID by control method | Existing CAN scheduler | `0x00c` for position, `0x00d` for velocity, `0x00e` for torque | Source inspection | done |
| T03 | Verify compile-time integration surface | MDK project source list | No generated/third-party changes; all edited sources listed by `.uvprojx` | Static checks | done |
| T04 | Validate frames and motion on a connected 6010 | Approved hardware test setup | Captured CAN trace and safe motion evidence | Real hardware procedure | blocked |

## Validation Plan

### Automated

- Static checks confirm the exact command IDs, little-endian field order, and all edited C++ sources are part of `wheel_leg_robort.uvprojx`.

### Manual / Evidence

- After the hardware freeze is lifted, capture one mode frame (`0x00b`, bytes `03 00 00 00 03 00 00 00`) and one position frame (`0x00c`) per node. Verify the node enters closed loop and the measured encoder position follows a bounded target.

## Blockers

- The ROADMAP freezes all real-machine bring-up and board-level tests. Packet/motion evidence is therefore unavailable.

## Execution Notes

- T01/T02: implemented in `firmware/stm32/Hardware/dvc_motor_dji.{h,cpp}` and `drv_can.{h,cpp}`. The scheduler keeps one buffer per node but now selects `0x00c` in position mode, `0x00d` in velocity mode, and `0x00e` in torque mode.
- T03: `git diff --check` passed. Static assertions confirmed the reference bytes for `Input_Pos=3.14` (`C3 F5 48 40`), position mode `3/3`, velocity mode `2/2`, `Input_Vel=10` (`00 00 20 41`), little-endian position-feedforward fields, node command-ID arithmetic, and inclusion of both changed sources in `wheel_leg_robort.uvprojx`.
- No local MDK compiler was available, so an MDK build was not run. This is non-blocking for source integration but does not replace T04 hardware evidence.
