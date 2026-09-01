# Phase 47 — Workspace Cleanup and Interface Freeze

Status: `complete`

## Goal and frozen decisions

Freeze the sole runtime as `Controller Core → ROS2 → MuJoCo`, remove all hardware runtime, retain
Simulink as read-only reference, and prove the cleanup preserves the Phase 46 numerical authority.
Phase 46 remains historical `REWORK`; this Phase does not redesign/tune WBC, QP or contact physics.

## Tasks

| ID | Task | Acceptance | Status |
| --- | --- | --- | --- |
| P47-T01 | Capture pre-cleanup baseline | frozen interpreter, build/tests, fresh primitive/slack replay | done |
| P47-T02 | Freeze current ROS runtime | WBC mode, shared profile, H0/reset, unique launch | done |
| P47-T03 | Remove hardware route | firmware/STM32 bridge deleted; docs archived with redirects | done |
| P47-T04 | Freeze interfaces and inventory | current path, lower-layer contract, legacy inventory | done |
| P47-T05 | Isolate historical runners | Phase34–46 targets default OFF and replayable with ON | done |
| P47-T06 | Post-cleanup regression | build/tests, ROS smoke, pre/post Phase46 equality | done |
| P47-T07 | Review and record | zero blocking findings, REVIEW PASS, RECORD created | done |

## Interface impact

- Adds `controller.mode=weighted_wbc`, the compiled nominal full-controller profile, current H0
  initialization and `current_weighted_wbc.launch.py`.
- Removes all STM32/serial public APIs.
- Does not add a ROS W_ref/W_WBC/W_MJ topic and does not change RobotState/TorqueCommand wire fields.

## Stop gate

Any material change in QP dimensions, W1–W6, solver status, W_ref, W_WBC, tau, slack, active-set
signature, frozen residuals or feasibility classification requires REVIEW=REWORK and no RECORD.
