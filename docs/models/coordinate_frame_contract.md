# Simulink reference–MuJoCo 坐标与接口契约

Status: `frozen — Phase 02 complete`

本文只冻结已有仿真证据支持的语义。历史 real-robot gate 已在 Phase 47 路线切换后退役。

## 1. 权威顺序

1. 已验证的 Simulink/Simscape baseline 行为与当前源码。
2. `source.slx` 的真实 frame 连接和 block 参数。
3. 本契约已经关闭的决定。
4. MuJoCo 导入 XML、CAD 名称和 mesh 外观仅作为候选证据。

统一的是跨系统边界的物理语义，不是两个仿真器内部的 frame 树。MuJoCo 可以保留便于建模和传感的局部 body/site frame。

## 2. Frame 与记号

### 2.1 Canonical navigation/world frame `{N}`

跨 Simulink reference、MuJoCo 和 Controller Adapter 的 canonical world 采用右手 FLU：

| Axis | 正方向 |
| --- | --- |
| `N_x` | 前 |
| `N_y` | 左 |
| `N_z` | 上，重力反方向 |

重力为 `g_N = [0,0,-g]`。MuJoCo 的 Z-up world 可直接采用该语义；Simscape baseline 不需要改模型，而是在其边界做显式旋转。

### 2.2 Simscape physical frame `{S}`

人工三视图、6-DOF Joint 连接和源码共同确认当前 baseline 为：`S_x` 前、`S_y` 上、`S_z` 右，重力沿 `-S_y`。它是右手系，因为 `S_x × S_y = S_z`。

普通向量从 Simscape 转到 canonical FLU：

```text
v_N = R_N_from_S * v_S

R_N_from_S = [1  0  0;
              0  0 -1;
              0  1  0]
```

`det(R_N_from_S)=+1`，逆变换是其转置。这个轴选择确实影响脚本，但 baseline 已显式处理：状态重构按 `[Sx,Sz,Sy]` 打包，三维动力学入口再用 `[1,3,2]` 恢复 `[Sx,Sy,Sz]`，重力也按物理 Y 轴使用。不得再机械交换一次。

### 2.3 Legacy Controller field order `{C_fields}`

现有 16-state 平移数组按 `[前,右,上]` 排列：

```text
v_Cfields = P_Cfields_from_S * v_S

P_Cfields_from_S = [1 0 0;
                    0 0 1;
                    0 1 0]
```

从 canonical FLU 看，同一字段打包为：

```text
v_Cfields = diag(1,-1,1) * v_N
```

两个打包矩阵的行列式均为 `-1`，所以 `{C_fields}` 只是兼容字段顺序，不是三维空间 frame。禁止将其直接用于 rotation、quaternion、cross product、angular velocity 或 torque/wrench 变换。

### 2.4 MuJoCo native world `{M}`

XML 数值、编译后轴向和人工三视图共同确认当前模型的 world 为 X 前、Y 左、Z 上。因此本 Phase 冻结：

```text
R_N_from_M = I
```

这只关闭 world 轴映射，不代表 `base_body` 原点、IMU 安装点、joint zero/sign 或真实机器已经通过。

### 2.5 Rotation 记号

`R_A_from_B` 将 `{B}` 表达的普通向量转换为 `{A}` 表达：

```text
v_A = R_A_from_B * v_B
R_B_from_A = transpose(R_A_from_B)
```

姿态采用 active rotation；矩阵列是被旋转 body frame 的轴在参考 frame 中的表达。四元数统一写 scalar-first `[w,x,y,z]`；比较和日志必须处理 `q` 与 `-q` 等价及符号连续性。

Simulink baseline 的原始姿态公式保持：

```text
R_S_from_Bs = Ry_S(yaw) * Rz_S(pitch) * Rx_S(roll)
```

其中 roll/pitch/yaw 分别绕 `+S_x/+S_z/+S_y`。映射到 `{N}` 后，正 yaw 是绕 `+N_z` 左转；正 pitch 的轴为 `+S_z=-N_y`。Adapter 必须做完整姿态基变换，不得把这些 Euler 数值直接当作标准 FLU Euler。

## 3. Base、CAD、COM 与 IMU frame

MuJoCo 中必须区分三个概念：

- `base_cad_frame`：当前 XML 的 `base_body` frame，即 SolidWorks 导出原点和姿态。保留它以免破坏整个子 body/geom/site 树。
- `base_control_frame`：姿态与 `base_cad_frame` 平行，原点位于机身刚体 `base_body` 的质心。它是后续 RobotState 的 base/body 候选。
- `imu_frame`：真实 IMU 安装 frame；其相对 `base_cad_frame` 或 `base_control_frame` 的 pose 必须由安装数据确定，不能用 COM 代替。

MuJoCo 已根据当前 geom 质量自动算出 nominal `base_body` COM，无需手工猜位置。当前 XML 下：

```text
base_body mass = 2.588 kg
p_baseCad_to_baseControl expressed in base_cad =
    [-0.077378152, 0.000000810, -0.032277680] m
```

Adapter 可用 `data.xipos[base_body]` 取机身刚体 COM 的 world 位置，并用 `data.xmat[base_body]` 取 CAD/control 平行 frame 的姿态；不要用惯性主轴姿态代替 body 姿态。整机 `subtree_com` 随关节构型变化，不是机身 base state。上述 COM 只定义 current nominal MuJoCo 模型。

当前 identity `base_frame` site 只是 `base_cad_frame` 的别名，不是已经批准的 COM frame 或真实 IMU frame。不得为了让 XML 原点“看起来在质心”而直接移动 `base_body`，否则所有子节点局部 pose 都需要同步补偿。

## 4. 位置、速度、加速度与 wrench

- Position 必须注明“哪个点相对哪个原点”以及表达 frame。
- Linear/angular velocity 和 acceleration 必须注明表达 frame。
- MuJoCo `accelerometer` 是 site-local specific force；不能无转换地作为 world kinematic acceleration。
- 同原点普通向量和力矩分别按 `v_A=R_A_from_B v_B`、`tau_A=R_A_from_B tau_B` 变换。
- 原点从 O 平移到 P 时必须显式加入 moment arm；禁止只旋转 wrench。
- 字段 pack 矩阵只允许用于已命名字段的 pack/unpack。

## 5. 公共状态契约

现有 baseline 16-state 保持兼容：

| Index | 字段 | 单位 | 当前语义 |
| ---: | --- | --- | --- |
| 1:3 | base position `[forward,right,up]` | m | legacy `{C_fields}`；当前 Simscape 6-DOF follower 原点相对 world |
| 4:6 | `[roll,pitch,yaw]` | rad | Simscape 轴序 `Rx/Rz/Ry`；yaw continuous |
| 7:9 | base linear velocity `[forward,right,up]` | m/s | world 物理量，legacy `{C_fields}` |
| 10:12 | Euler rates `[rollRate,pitchRate,yawRate]` | rad/s | 参数率，不等同于未注明 frame 的 gyro 三轴 |
| 13:14 | `[xi_L,xi_R]` | m | wheel center 相对 base-forward 的几何位置 |
| 15:16 | `[dxi_L,dxi_R]` | m/s | 上述几何量的时间导数 |

新 RobotState 的 world 向量应直接按 canonical FLU `[x_forward,y_left,z_up]` 表达；与旧数组的转换只能位于命名清晰的兼容 Adapter 中。精确消息 schema 留给 Phase 03。

`xi` 不是 wheel spin angle。差模符号保持：

```text
xi_common = (xi_L + xi_R)/2
xi_delta  = (xi_R - xi_L)/2
```

## 6. Command 与 joint 顺序

- Wrench block：left before right；单侧 `[Fx,Fy,Fz,Tx,Ty,Tz]`。
- 公共驱动 joint：`[left_hip,left_knee,left_wheel,right_hip,right_knee,right_wheel]`。
- connect1/connect2 闭链辅助 joint 不进入公共 RobotState/TorqueCommand。
- `W* = W_mpc + slack` 符号不变。
- 六个 Simscape 驱动 joint 的正轴均为 `+S_z=-N_y`；MuJoCo qpos0 下均为 `+N_y`。因此 `q_C=-q_M+b_joint`、`dq_C=-dq_M`、`tau_M=-tau_C`，左右两侧不增加镜像负号。
- `b_joint` 必须在 Phase 04 用 matching pose + 第二姿态 FK 回归逐关节冻结；真实 encoder offset/torque 方向在 Phase 06 低风险验证。详见 Phase 证据 `joint_coordinate_mapping.md`。

## 7. MuJoCo 使用规则

- 允许使用 local body/site frame 生成传感器和任务量。
- 每个 Adapter 输出必须可追溯为 `native quantity -> named transform -> canonical quantity -> optional legacy pack`。
- 保留 `base_body` CAD frame；`base_control_frame` 是 current state authority。不新增未被 MuJoCo runtime 使用的硬件 sensor frame。
- 需要 world position/velocity 时，优先读取明确 object/site pose，再映射到 `{N}`。
- 在 joint zero/sign 关闭前，不旋转 mesh、不改 joint axis/zero、不删除 weld；验证使用候选副本或显式测试场景。

## 8. Decision gates

| Gate | 状态 | 结论/剩余证据 |
| --- | --- | --- |
| DG01 | closed | `{N}`=FLU；`{S}`=前上右；显式 `R_N_from_S` |
| DG02 | closed | canonical base 原点冻结为 current nominal torso COM，site 已落地 |
| DG03 | closed | active `[w,x,y,z]`、Simscape→FLU 共轭变换和跨 ±π yaw round-trip 已测试 |
| DG04 | retired | historical IMU gate removed by the MuJoCo-only route |
| DG05 | closed | joint 顺序、轴、zero offset 和 `q/dq/tau` 符号以 current Adapter 为准 |
| DG06 | closed | 人工三视图与数值 FK 确认 `{M}`=FLU，`R_N_from_M=I` |
| DG07 | closed | 坐标修正使用辅助 site + Adapter 足够；本 Phase 不需要 CAD 重导出 |
| DG08 | closed | `conda:mujoco` + 冻结 environment + 动态探针 |

历史真机坐标 gate 已退役，不构成未完成的 current requirement。

## 9. 当前结论

跨系统统一采用 FLU canonical world。Simulink baseline 保留其 X 前、Y 上、Z 右的内部物理坐标，通过 `R_N_from_S` 进入统一边界；MuJoCo 保留原生 Z-up 和方便的局部 body/site frame，其 world 已与 canonical FLU 同轴。Controller 旧 `[前,右,上]` 数组只作为兼容字段排列。base CAD 原点、机身 COM 和 IMU 安装点是三个独立概念，必须分别命名和验证。
