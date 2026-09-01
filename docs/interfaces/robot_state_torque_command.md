# Lower-Layer Interface Contract

Status: `frozen — Phase 47`

## Public ROS boundary

```text
MuJoCo Adapter → RobotState → Controller Core → TorqueCommand → MuJoCo Adapter
```

所有量使用 SI。世界系 `{N}` 为 X forward、Y left、Z up；`B` 是
`base_control_frame`，原点为 torso rigid-body COM。固定关节顺序为：

```text
[left_hip,left_knee,left_wheel,right_hip,right_knee,right_wheel]
```

### RobotState

| Field | Unit/order | Frozen semantic |
| --- | --- | --- |
| `sample_time_ns` | ns | MuJoCo source monotonic time；reset 后开启新 epoch；同一 epoch 严格递增 |
| `base_position_n_m` | m, `[Nx,Ny,Nz]` | `B` 原点在 `{N}` 的位置 |
| `q_n_from_b` | `[w,x,y,z]` | 把 `B` 表达向量主动旋转到 `{N}` 的单位四元数 |
| `base_linear_velocity_n_m_s` | m/s, `[Nx,Ny,Nz]` | `B` 原点速度，在 `{N}` 表达 |
| `base_angular_velocity_n_rad_s` | rad/s, `[Nx,Ny,Nz]` | `B` 相对 `{N}` 角速度，在 `{N}` 表达 |
| `joint_position_rad` | rad, six-joint order | canonical output-joint coordinate |
| `joint_velocity_rad_s` | rad/s, six-joint order | canonical coordinate derivative |
| `contact_state` | `[left,right]` | `0 unknown / 1 no-contact / 2 contact` |

所有数值必须 finite；四元数 norm tolerance 默认 `1e-6`。ROS quaternion 字段为
`x,y,z,w`，只有 `wheel_leg_ros::toRos/fromRos` 可做顺序转换。

### TorqueCommand

| Field | Unit/order | Frozen semantic |
| --- | --- | --- |
| `source_sample_time_ns` | ns | 生成命令的 RobotState source timestamp |
| `joint_torque_nm` | N·m, six-joint order | canonical output-axis torque；全部 finite |

Adapter 按 `data->ctrl[actuator_id] = -joint_torque_nm[index]` 写入 MuJoCo；这个负号
来自 canonical joint 与 MuJoCo actuator coordinate 的冻结映射。Core 负责每关节
`{10,10,2,10,10,2}` N·m saturation，Adapter 负责 stale/timeout fail-to-zero。

## Internal WBC interfaces

### W_ref

- Actor：当前 fixed reference；未来为 12X/16X NMPC。
- Receiver：`ControllerCore::stepWeightedWbc` / `WeightedWbcController`。
- Storage：`WbcReference::interaction_wrench_flu[12]`，不建立 ROS message。
- Order：`[L_Fx,L_Fy,L_Fz,L_Tx,L_Ty,L_Tz,R_Fx,R_Fy,R_Fz,R_Tx,R_Ty,R_Tz]`。
- Frame：controller body FLU；moment origin 为对应 wheel-body origin。
- Sign：wheel 对 leg/base 的 follower wrench；force N，moment N·m。
- Timestamp：与生成该 reference 的同一 accepted RobotState snapshot 关联。

### W_WBC

`WeightedWbcController::step` 从 authoritative physical solution 重建的实际选择 wrench，
不是 raw solver latent/null component。frame、origin、sign、order 和单位与 W_ref 相同。
signed slack 仅表示 interaction-wrench fidelity，并满足：

```text
W_WBC - W_ref - signed_slack = interaction_residual
```

### W_MJ

MuJoCo 实际 contact wrench 仅用于诊断/oracle：`efc/contact rows → row reaction → Cartesian
point force → per-wheel aggregate wrench → production reference`。其 order/frame/origin/sign 与
W_ref 相同，但不作为 current ROS topic 或 Controller feedback channel。

### Planner → xi_ref

wheel-position planner 的 common/differential position、velocity、acceleration reference 保持
Core 内部接口；Phase 47 不改变其单位、方向或调度，也不把它发布为 ROS API。

## Timing and reset

- MuJoCo step 2 ms；每 5 step 发布一次 RobotState，WBC period 10 ms。
- Core first accepted sample 的 `dt=0`；之后从连续 source timestamps 计算。
- rejected sample 不推进 Core history；source clock rollback 必须 reset。
- Adapter receipt-time watchdog 为 100 ms，最大 source lag 为 50 ms。
- current H0 reset 后先取得首个有效 torque，再进入 MuJoCo physical stepping。
- simulation reset 后必须再调用 controller reset；两侧都完成后新 epoch 才能继续。

`W_ref`、`W_WBC`、slack、solver/rank/residual 是内部诊断契约；公共 ROS 边界仍只有
RobotState 和 TorqueCommand。
