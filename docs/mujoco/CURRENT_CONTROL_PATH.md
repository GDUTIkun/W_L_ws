# Current Control Path

Status: `frozen — Phase 47`

## Runtime authority

唯一 current runtime 是：

```bash
cd ros_ws
source install/setup.bash
ros2 launch wheel_leg_mujoco current_weighted_wbc.launch.py
```

该 launch 固定使用 `current_weighted_wbc.yaml`、
`scene_axisymmetric_centered_com_v1.xml`、500 Hz MuJoCo step、100 Hz
RobotState/WBC、floating base、command application 和 `current_weighted_wbc_h0`。

## Data and call path

```text
mujoco_node / Adapter::extractState
  → RobotState ROS message
  → ControllerNode::onState
  → ControllerCore::step
  → ControllerCore::stepWeightedWbc
  → WeightedWbcController::step
  → NominalWbcModel::evaluate
  → WeightedWbcProblem::assemble (42D reduced QP)
  → DenseQpSolver
  → physical_solution[12:18] = tau
  → TorqueCommand ROS message
  → Adapter::acceptCommand / writeControls
  → data->ctrl[actuator_id] = -tau
  → mj_step
```

`W_ref` 是 Core 内部 `WbcReference::interaction_wrench_flu`；fixed reference 与未来
NMPC override 必须共用它。`W_WBC` 由 `WeightedWbcController::step` 根据 authoritative
interaction map 重建。`W_MJ` 仅由 Phase 44–46 replay/oracle 通过 MuJoCo contact/efc rows
重建，不是 ROS 控制 topic。

## Non-current entry points

- `weighted_wbc_loop`：确定性 regression/oracle，不是部署入口。
- `zero_loop.launch.py`：ROS transport smoke，不是 current controller。
- Phase 34–46 runner：冻结历史诊断，只有配置
  `-DWHEEL_LEG_BUILD_LEGACY_RUNNERS=ON` 时构建。
- Simulink baseline：只读参考，不是 production runtime。
