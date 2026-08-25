# Simulink–MuJoCo–Real 坐标与接口契约

Status: `draft — Phase 02 active`

本文件先冻结已有证据能够支持的语义；标为 `GATE` 的项目在动态或真机证据完成前不得假装已经统一。

## 1. 权威顺序

1. 已验证的 Simulink/Simscape baseline 行为与当前源码。
2. `source.slx` 的真实 frame 连接和 block 参数。
3. 本契约已经关闭的决定。
4. MuJoCo 导入 XML、CAD 名称和 mesh 外观仅是候选证据。

MuJoCo 可以保留局部 body/site frame。统一的对象是跨系统边界的物理语义，不是强迫两个仿真器拥有相同的内部 frame 树。

## 2. Frame 与记号

### 2.1 Canonical physical frame `{S}`

Phase 02 当前采用已验证 Simscape 物理 frame 作为 canonical physical frame：

| Axis | 方向 | 备注 |
| --- | --- | --- |
| `S_x` | 前向 | base/腿纵向 |
| `S_y` | 向上 | gravity 的反方向 |
| `S_z` | 右侧 | 右髋为 +0.2 m，左髋为 -0.2 m |

`{S}` 是右手系，`S_x × S_y = S_z`。正 roll/pitch/yaw 分别对应物理 X/Z/Y 轴；正 yaw 从 +X 朝 -Z，表示左转。

### 2.2 Controller field order `{C_fields}`

现有 Controller/NMPC 平移数组按 `[前向, 右侧, 向上]` 排列：

```text
v_Cfields = P_Cfields_from_S * v_S

P_Cfields_from_S = [1 0 0;
                    0 0 1;
                    0 1 0]
```

`det(P_Cfields_from_S)=-1`，所以 `{C_fields}` 只是字段顺序，不是可用于三维几何的 frame。禁止用该矩阵直接转换 rotation、quaternion、cross product、angular velocity 或 torque pseudovector。需要空间运算时必须先回到 `{S}` 或另一个明确的右手 frame。

### 2.3 MuJoCo native frame `{M}`

旧模型当前候选为 X 前向、Y 左侧、Z 向上的右手系。待 X 正向动态确认后，候选旋转为：

```text
v_S = R_S_from_M * v_M

R_S_from_M = [1  0 0;
              0  0 1;
              0 -1 0]
```

`det(R_S_from_M)=+1`。在 DG06 关闭前，Adapter 不得把这个候选矩阵写成已批准常量。

### 2.4 Rotation 记号

`R_A_from_B` 将在 `{B}` 表达的普通向量转换为 `{A}` 表达：

```text
v_A = R_A_from_B * v_B
R_B_from_A = transpose(R_A_from_B)
```

姿态采用 active rotation 解释；矩阵列是被旋转 frame 的轴在参考 frame 中的表达。Simulink baseline 的精确姿态公式保持为：

```text
R_S_from_B = Ry(yaw) * Rz(pitch) * Rx(roll)
```

四元数统一为 scalar-first `[w,x,y,z]`。Simscape 的 `[S,V]` 与 MuJoCo 的 `[w,x,y,z]` 元素顺序一致，但仍须验证两者表示的旋转方向后再直接复制；四元数 `q` 与 `-q` 表示同一姿态，比较和日志必须处理符号连续性。

## 3. 位置、速度、加速度与 wrench

- Position 必须写明“哪个点相对哪个原点”以及表达 frame。
- Linear/angular velocity 必须写明表达 frame；不能只写 `vx` 或 `gyro`。
- MuJoCo `accelerometer` 是 site-local、包含 gravity 的加速度量，不能无转换地冒充 Controller world linear acceleration。
- 同原点的普通向量：`F_A = R_A_from_B F_B`。
- 同原点的力矩：`tau_A = R_A_from_B tau_B`。
- 原点从 O 平移到 P 时，必须显式加入 moment arm；禁止只旋转 wrench。
- 任何使用 `P_Cfields_from_S` 的地方只能是已命名字段的 pack/unpack，不能作为 wrench 的统一空间变换。

## 4. 公共状态契约

现有 16-state baseline 保持：

| Index | 字段 | 单位 | 参考/表达语义 |
| ---: | --- | --- | --- |
| 1:3 | base position `[forward,right,up]` | m | base reference point 相对 world；`C_fields` 序列化 |
| 4:6 | `[roll,pitch,yaw]` | rad | `R_S_from_B = Ry*Rz*Rx`；yaw continuous |
| 7:9 | base linear velocity `[forward,right,up]` | m/s | world `{S}` 物理量，`C_fields` 序列化 |
| 10:12 | Euler rates `[rollRate,pitchRate,yawRate]` | rad/s | 参数率，不等同于未说明 frame 的 gyro 三轴 |
| 13:14 | `[xi_L,xi_R]` | m | wheel center 相对 base-forward 的几何位置 |
| 15:16 | `[dxi_L,dxi_R]` | m/s | 上述几何量的时间导数 |

`xi` 不是 wheel spin angle，terrain contact frame 不得重定义它。差模 canonical sign 保持：

```text
xi_common = (xi_L + xi_R)/2
xi_delta  = (xi_R - xi_L)/2
```

## 5. 公共 command 与 joint 顺序

- Wrench block 顺序：left before right。
- 单侧字段顺序：`[Fx,Fy,Fz,Tx,Ty,Tz]`。
- 公共驱动 joint 顺序：`[left_hip,left_knee,left_wheel,right_hip,right_knee,right_wheel]`。
- 辅助闭链 connect joint 不进入公共 RobotState/TorqueCommand，只允许作为 MuJoCo 内部诊断量。
- Simulink 左右驱动关节均以 `[hip,knee,wheel]` 使用同一控制初值 `leg.q0`；hip 的 Simscape joint target 是 `leg.q0(1)-pi/2`。
- `W* = W_mpc + slack` 的符号不变。

`GATE DG05`：每个 joint 的 MuJoCo q、真机 encoder q、Controller q 与 torque 正方向尚需正微扰/低风险实物验证；在此之前不得通过数组重排或负号“修到看起来能跑”。

## 6. MuJoCo 使用规则

- 允许使用 local body/site frame 生成 sensor 和任务量。
- 每个 Adapter 输出必须可追踪为 `native quantity -> named transform -> canonical physical quantity -> interface field pack`。
- `base_frame` site 可继续作为局部 IMU 候选；必须记录 site 相对 base reference 的 pose。
- 需要 world-frame position/velocity 时优先使用明确的 frame sensor 或 MuJoCo object pose，再映射到 `{S}`；不复制 Simscape 中难用的 world-fixed sensor 连线形式。
- 原始 `wheel_leg.xml` 在 gate 关闭前不旋转 mesh、不改 joint axis/zero、不删 weld；验证使用候选副本或显式测试场景。

## 7. 尚未关闭的 gates

| Gate | 待确认 | 放行证据 |
| --- | --- | --- |
| DG02 | base reference 原点与真实 IMU 安装点 | 模型 site/COM/reference 数值表 + 真机安装测量 |
| DG03 | Simscape Q 与 MuJoCo framequat 的旋转方向、continuous yaw | 0°/90°/跨 ±π round-trip 测试 |
| DG04 | gyro/accelerometer 与 Controller acceleration 的转换 | 静止、单轴角速度和已知线加速度测试 |
| DG05 | 6 个驱动 joint 的 zero/sign/torque/rolling | q 正微扰、wheel center 位移、真机低风险核对 |
| DG06 | `{M}` X 正向和 `R_S_from_M` | MuJoCo viewer frame + 数值 FK/axis 输出 |
| DG07 | XML frame/site 是否足以修正旧 CAD 导入 | 动态测试；不足时才给出重导出操作单 |
| DG08 | 正式 MuJoCo 运行环境 | 可复现环境文件、版本和动态测试命令 |

## 8. 当前结论

Simulink 与 MuJoCo 不需要共享相同的内部父子 frame 结构。统一方案是：以 Simscape 已验证的 `{S}` 物理语义为基准，保留 MuJoCo 的 Z-up body/site 优势，在 Adapter 中做命名清楚、可测试、可逆的右手 frame 变换；Controller 旧数组 `[前向,右侧,向上]` 只作为兼容字段顺序保留。

