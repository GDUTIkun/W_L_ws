# Legacy Interface Inventory

| Item | Historical purpose | Current reachable | Evidence dependency | Status | Deletion condition |
| --- | --- | --- | --- | --- | --- |
| `weighted_wbc_loop` | deterministic WBC/MuJoCo formal runner | NO | YES | FROZEN ORACLE | remove only after an equivalent deterministic oracle is approved |
| Phase34–46 runner targets/configs | attribution and repair evidence replay | NO (default OFF) | YES | FROZEN | remove only when historical evidence is intentionally retired |
| Phase34–46 tools/evidence | machine-readable historical conclusions | NO | YES | ARCHIVED IN PLACE | never remove during ordinary cleanup |
| `zero_loop.launch.py` | ROS transport smoke | NO | YES | DEPRECATED SMOKE | remove when current launch has an equivalent transport fault suite |
| planar/3D standing loops | earlier controller regression | NO | YES | FROZEN ORACLE | remove only with replacement regression coverage |
| Simulink baseline | algorithm/numerical comparison | NO | YES | READ-ONLY REFERENCE | explicit new route decision only |
| hardware technical documents | real-robot design history | NO | YES | ARCHIVED | explicit history-retention decision only |
| STM32 firmware and ROS bridge | retired hardware runtime | NO | NO | DELETED | not applicable |

CURRENT is limited to `wheel_leg_core`, `wheel_leg_msgs`, `wheel_leg_ros`, `wheel_leg_mujoco`,
`current_weighted_wbc.yaml`, the current scene and `current_weighted_wbc.launch.py`. Diagnostic-only
W_MJ reconstruction remains in historical replay tools and cannot be called by the current runtime.
