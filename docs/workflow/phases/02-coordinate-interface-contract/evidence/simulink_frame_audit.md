# Simulink/Simscape 坐标与传感语义审计

Status: `evidence captured`

## 证据来源

- 模型：`simulation/simulink_baseline/model/simulate/two_legs/source.slx`
- MATLAB：R2024b，`24.2.0.2712019`
- 自动清单：`simulink_frame_manifest.json`
- 生成脚本：`tools/maintenance/inspect_simulink_frames.m`
- 当前用户批准模型 SHA256 为 `B4036771E6F01614A2F85E0C0D980C24E56C2CEC7372DEA1AE31832EEB6BE279`。
- 最新 manifest 覆盖用户有意断开的 `PD_only/6-DOF Joint` LConn2–LConn7；模型加载回调报告 `Dirty on -> on`，inspector 本身未保存 `source.slx`。

MathWorks 的 6-DOF Joint 定义说明：Px/Py/Pz 分别是 follower 相对 base、沿 base frame X/Y/Z 轴的位移；球副 Q 是 follower 相对 base 的四元数，角速度按所选 resolution frame 表达。参见 [6-DOF Joint](https://www.mathworks.com/help/sm/ref/6dofjoint.html) 与 [Rotational Measurements](https://www.mathworks.com/help/sm/ug/rotational-measurements.html)。

## 基座 frame 拓扑

清单中的 `PortConnectivity` 给出以下真实连接：

```text
World Frame1
  -> Rigid Transform
       translation: +Y, base.simscapeWorldYOffset - ctrl.commonModeContactPreload
       rotation: None
  -> 6-DOF Joint / Base frame
  -> 6-DOF Joint / Follower frame
  -> base solid、左右髋关节支路及外力块
```

因此，6-DOF Joint 的 Base frame 与 Simscape World frame 同轴，仅有 Y 向原点平移；其 Px/Py/Pz、线速度和球副姿态均是基座 follower 相对该 world-aligned base frame 的量。

## 物理轴与 Controller 字段顺序

当前源码明确规定 Simscape 物理轴：

| Simscape 物理轴 | 语义 | 模型证据 |
| --- | --- | --- |
| +X | 机身前向 | `controller_attitude_kinematics.m`；腿段 Rigid Transform 沿 +X |
| +Y | 竖直向上 | `controller_attitude_kinematics.m`；World-to-base 初始偏置沿 +Y |
| +Z | 机身右侧 | 右髋 Rigid Transform 为 +Z 0.2 m，左髋为 -Z 0.2 m |

这组轴是右手系。Controller/NMPC 的平移字段却按 `[前向, 右侧, 向上]` 序列化，即：

```text
[controller X; controller Y; controller Z]
  = [Simscape X; Simscape Z; Simscape Y]
```

`spatial_two_leg_qp_core.m` 在进入物理空间动力学时使用 `state([1,3,2])` 和 `state([7,9,8])` 恢复 Simscape 物理顺序。该交换是字段排列，不是空间旋转；禁止把它直接用于叉乘、姿态、角速度或力矩变换。

## 6-DOF Joint 实际测量

| 6-DOF 输出 | 下游 | 当前语义 | 单位/表达 frame |
| --- | --- | --- | --- |
| Px | PS-Simulink Converter4 -> Mux input 2 | base 前向位置 | m，World/Base +X |
| Vx | PS-Simulink Converter3 -> Mux input 5 | base 前向速度 | m/s，World/Base +X |
| Py | PS-Simulink Converter6 -> Mux input 3 | base 高度坐标 | m，World/Base +Y |
| Vy | PS-Simulink Converter7 -> Mux input 6 | base 竖直速度 | m/s，World/Base +Y |
| Pz | PS-Simulink Converter8 -> Base Lateral State input 1 | base 右向位置 | m，World/Base +Z |
| Vz | PS-Simulink Converter5 -> Base Lateral State input 2 | base 右向速度 | m/s，World/Base +Z |
| Q | PS-Simulink Converter18 -> roll/pitch/yaw Fcn | follower 相对 base 的姿态 | `[S,Vx,Vy,Vz]`，无量纲 |
| w | PS-Simulink Converter19 -> angular velocity split | base 相对 world 的角速度 | rad/s，`SphSensingFrame=BaseFrame` |

模型参数还确认：

- Px/Py/Pz 的 Position 与 Velocity sensing 均开启，Acceleration sensing 关闭。
- 球副 Position 与向量 Velocity sensing 开启，Acceleration sensing 关闭。
- `SphSensingFrame=BaseFrame`，`SphVelocityTargetInFollowerFrame=off`。
- Q 的初值为 `base.initialQuaternion`，MathWorks 四元数顺序为标量在前 `[S,V]`。
- 模型没有 Transform Sensor；基座状态直接来自 6-DOF Joint。
- 两个 Spatial Contact Force block 的保存状态下 sensing 选项均关闭，不能把它们当作当前公共传感器接口。

## 姿态与 yaw

控制源码使用：

```text
R = Ry(yaw) * Rz(pitch) * Rx(roll)
```

其中 roll/pitch/yaw 分别关联 Simscape 物理 X/Z/Y 轴。`turning_world_reference.m` 进一步冻结：正 yaw 朝 Controller 负侧向，也就是从物理 +X 朝 -Z 转动；结合“+Z 为右侧”，正 yaw 表示左转。

模型内四元数到 roll/pitch/yaw 的三个 Fcn block 已记录在 JSON 清单中。continuous yaw 的跨 ±π 展开仍由状态重构代码完成，不应只依赖这三个瞬时 `atan2` 输出。

## 驱动关节与公共顺序

| Side | 公共关节顺序 | Simulink block | 初始 target | Sensing/Actuation |
| --- | --- | --- | --- | --- |
| Right | hip | `Right Revolute Joint` | `leg.q0(1)-pi/2` | q/dq on，torque input |
| Right | knee | `Revolute Joint1` | `leg.q0(2)` | q/dq on，torque input |
| Right | wheel | `Revolute Joint2` | `leg.q0(3)` | q/dq on，torque input |
| Left | hip | `Left Revolute Joint3` | `leg.q0(1)-pi/2` | q/dq on，torque input |
| Left | knee | `Revolute Joint4` | `leg.q0(2)` | q/dq on，torque input |
| Left | wheel | `Revolute Joint5` | `leg.q0(3)` | q/dq on，torque input |

Controller/WBC 公共数组仍采用 left block before right block；表格先按模型支路列出，不改变公共接口顺序。Revolute Joint 的正方向遵循其 Base/Follower frame 和右手轴定义，参见 [Revolute Joint](https://www.mathworks.com/help/sm/ref/revolutejoint.html)。仅凭 block 名称和零位表达式仍不足以批准 MuJoCo/真机 encoder 正方向；该项保留到关节微扰核对。

## 尚需人工/动态证据的项目

- 在 Mechanics Explorer 中显示 World、base、左右 hip frame，确认视觉轴与上述数值拓扑一致。
- 对左右 hip/knee/wheel 分别施加小正角度，记录 wheel center/连杆实际运动方向。
- 将真机 encoder `q>0`、`dq>0` 和安全低速 torque 正方向与同一语义表对齐。
- 验证 IMU 的安装 frame、输出四元数含义和加速度是否为 specific force；这些不能从当前 `source.slx` 推断。

## 2026-08-25 人工证据复核

`evidence/manual/正视.png`、`右视.png`、`俯视.png` 已从三个视角确认 Simscape 物理轴为 X 前、Y 上、Z 右；`word2body.png` 确认 World 到 6-DOF Joint base 之间没有旋转，只有 +Y 平移。Rigid Transform5/10 已由自动 manifest 覆盖且用户再次确认，因此不要求重复截图。

这组轴会影响脚本，但现有 baseline 已一致处理：`full_base_nmpc_state_signal.m` 将平移量打包为 `[Sx,Sz,Sy]`，`spatial_two_leg_qp_core.m` 再用 `[1,3,2]` 恢复物理 `[Sx,Sy,Sz]`，`controller_attitude_kinematics.m` 使用物理 X/Z/Y 的 roll/pitch/yaw 轴。结论是保留 baseline，不再额外交换；在 Adapter 边界用 `R_N_from_S` 转到 FLU。

复查工作树版本时发现 `source/PD_only/6-DOF Joint` 的 LConn2–LConn7 已断开。用户于 2026-08-25 确认该修改有意；manifest 已按当前模型重生成。坐标参数和右侧测量连接未变，随后 5 s smoke 通过。
