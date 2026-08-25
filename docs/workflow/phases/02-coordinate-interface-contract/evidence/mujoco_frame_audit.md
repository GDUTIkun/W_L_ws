# 旧 MuJoCo 模型 frame/joint/sensor 审计

Status: `import facts captured; convention not approved`

## 可重复入口

```powershell
& tools/maintenance/audit_mujoco_frames.ps1
```

输出为 `mujoco_frame_manifest.json`。当前静态审计结果：11 个 body、10 个 hinge joint、5 个 site、19 个 sensor，公共名称无重复。

动态编译入口：

```powershell
conda run --no-capture-output -n mujoco `
  python tools/maintenance/audit_mujoco_runtime.py
```

环境由 `simulation/mujoco/environment.yml` 冻结为 Python 3.12 + MuJoCo 3.12.0，输出为 `mujoco_runtime_manifest.json`。

## 原生 frame 树

```text
world
└─ base_body (pos 0 0 0.6, freejoint, but welded to world)
   ├─ base_frame site (identity pose in base_body)
   ├─ right_thigh_body (local euler -pi/2 0 -pi/2)
   │  ├─ right_calf_body (local euler 0 0 pi)
   │  │  └─ right_wheel_body
   │  └─ right_connect1_body
   │     └─ right_connect2_body
   └─ left_thigh_body (local euler -pi/2 0 -pi/2)
      ├─ left_calf_body (local euler 0 0 pi)
      │  └─ left_wheel_body
      └─ left_connect1_body
         └─ left_connect2_body
```

`compiler angle="radian"`，未显式写 `eulerseq`，因此使用 MJCF 默认 intrinsic `xyz`。MJCF 的 body pose 是相对父 body 的局部 pose，joint axis 是定义该 joint 的局部 body frame 中的轴。参见 MuJoCo [XML Reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html)。

## 导入轴候选

从导入几何的数值符号可得到以下候选，而不是最终批准结论：

| MuJoCo native axis | 当前导入迹象 | 置信度 |
| --- | --- | --- |
| X | 机身纵向/前后 | 中；需 viewer 与 FK 微扰确认正向 |
| Y | +Y 为左侧 | 高；left thigh Y=+0.04425，right thigh Y=-0.04425 |
| Z | +Z 向上 | 高；base 初始 Z=0.6，scene gravity 为 `[0,0,-9.81]` |

如果 X 正向最终确认为前向，则 MuJoCo native `{M}` 是常见的 forward-left-up 右手系；到 Simscape 物理 `{S}`（forward-up-right）的候选旋转为：

```text
S_x =  M_x
S_y =  M_z
S_z = -M_y
```

该变换必须通过动态方向性测试后才能进入 Adapter。

## Joint 与 sensor 事实

- 6 个驱动候选：left/right hip、knee、wheel；另有 4 个 connect1/connect2 闭链辅助 joint。
- 所有 10 个 hinge 均显式写为局部 `axis="0 0 1"`。由于父 body 含局部 Euler 旋转，“XML 都写 Z 轴”不代表它们在 world 中同轴或同号。
- 当前 XML 没有 actuator；jointpos/jointvel 只说明可观测，不能证明 torque command 映射已存在。
- `base_frame` site 没有独立 orientation，继承 `base_body` 姿态。
- `base_accel` 与 `base_gyro` 均绑定 `base_frame` site；它们在 site 局部坐标输出。MuJoCo accelerometer 输出包含 gravity 的 site 线加速度，gyro 输出 site 局部角速度，参见 [accelerometer/gyro](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-accelerometer)。
- `base_quat` 是 `base_frame` 在 global coordinates 中的单位四元数；MuJoCo 四元数顺序为 `[w,x,y,z]`，参见 [framequat](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-framequat) 与 [Simulation](https://mujoco.readthedocs.io/en/latest/programming/simulation.html)。

## MuJoCo 3.12.0 编译与方向性探针

旧 scene 能成功编译，但编译后的关键事实为：

| Item | Compiled result |
| --- | --- |
| nq / nv | 17 / 16 |
| actuator `nu` | 0 |
| sensor count / data width | 19 / 26 |
| timestep | 0.002 s |
| gravity | `[0,0,0]`，不是 scene 文件表面的 `[0,0,-9.81]` |
| base qpos0 | position `[0,0,0.6]`，quaternion `[1,0,0,0]` |

因此 gravity 冲突已经从“静态风险”升级为真实 compiled finding：include 中 `wheel_leg.xml` 的 zero gravity 最终生效。坐标/IMU 测试必须在运行时显式注入 gravity 或先通过独立模型修改 gate，不能按 `scence.xml` 文本假定重力已经开启。

qpos0 下，左右 hip/knee/wheel 的 compiled world axis 都约为 native `+Y`。对每个驱动 joint 做 `q += 1e-6 rad` 中心差分后：

- left/right hip 与 knee 的 wheel-center 位移导数数值同号，说明导入树的左右 q 正方向当前相同，而不是镜像负号。
- wheel joint 正微扰不移动 wheel center，只改变 wheel orientation，符合 spin joint 预期。
- native +Y 轴是左右轮轴方向；若 native +X 冻结为前向，则按右手规则，正 wheel q 对应正向 rolling 候选。

运行时 sensor probes（不改 XML，仅禁用 equality 并在内存注入状态）得到：

- 注入 gravity `[0,0,-9.81]` 的自由落体中，base qacc Z 约为 `-9.80998 m/s²`，accelerometer 约为零，确认其行为是 site-local specific force，而不是可直接当作 world kinematic acceleration 的量。
- 注入 base `+90°` native-Z quaternion `[0.7071,0,0,0.7071]`，`framequat` 原样报告同一 `[w,x,y,z]`。
- 注入 native +Z angular rate `0.25 rad/s`，gyro 在 identity site frame 报告 `[0,0,0.25]`。

## 当前风险与 intended mapping

| 当前事实 | 风险 | 本 Phase intended handling |
| --- | --- | --- |
| `base_body` 同时有 freejoint 和 world weld | 动态上仍被固定，不能用于 floating-base sensor 验证 | 暂不改 XML；动态测试候选副本中明确选择 fixed 或 free |
| `wheel_leg.xml` gravity=0，`scence.xml` gravity=-9.81 | MuJoCo 3.12.0 compiled gravity 已确认为零 | 后续独立 Phase/受控修改修正；本 Phase 的 sensor probe 在内存显式注入 |
| nested Euler + local joint Z axes | joint 正方向不直观 | 对每个 joint 做 `q += epsilon` FK/axis 微扰 |
| base site identity | 方便但没有 canonical 注释 | 保留局部 site；Adapter 明确 `M -> S -> interface fields` |
| 无 actuator | 不能验证 torque 正方向或闭环 | 在后续冻结的测试副本/Adapter 中补，不在本次静态审计擅改 |
| 闭链 connect joints 也有 encoder | 公共 joint 数组可能误收辅助关节 | 公共接口只列 hip/knee/wheel；辅助 joint 仅内部诊断 |

## 暂不修改原模型的理由

当前尚未关闭 joint zero/sign、MuJoCo X 正向、compiled gravity 和动态运行环境四个 gate。此时旋转 mesh、改 joint axis、删 weld 或覆盖 Euler 会把“导入事实”和“批准语义”混在一起。第一步应在候选副本中加载并做 frame/FK 微扰；验证通过后，再决定只加辅助 site/注释，还是需要 CAD/GUI 重导出。
