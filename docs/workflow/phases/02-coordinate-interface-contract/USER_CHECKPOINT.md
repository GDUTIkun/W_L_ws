# Phase 02 用户坐标方向核对单

Status: `Simulink/MuJoCo world-axis evidence reviewed; real joint/IMU deferred`

2026-08-25 回传审查见 `evidence/manual/REVIEW.md`。A/B 的 world 轴与 frame 证据已通过；C 未执行，并按用户决定在 MuJoCo frame/site 落地后转入真机验证。该延期不等同于 DG05 或真实 IMU PASS。

自动审计已经确认 frame 拓扑、字段排列、旧 MJCF joint axis、compiled gravity 和基础 sensor 行为。本单只要求 GUI/实物才能可靠确认的项目，不要求你修改 `source.slx` 或原始 `wheel_leg.xml`。

## A. Simulink / Mechanics Explorer

1. 在 MATLAB R2024b 中运行：

   ```matlab
   cd('D:\Workspace\W_L_ws\simulation\simulink_baseline')
   open_proformance_test(false);
   open_system('source/PD_only')
   ```
2. 截图 `World Frame1 -> Rigid Transform -> 6-DOF Joint`，确保能看见 block 名和连接线。
3. 打开 `Rigid Transform` 参数，截图其 `Translation: +Y`、无 rotation。
4. 打开 `Rigid Transform5` 与 `Rigid Transform10`：

   - `Transform5` 应为 `+Z, 0.2 m`，连接右髋；
   - `Transform10` 应为 `+Z, -0.2 m`，连接左髋。
5. 运行一个已有短仿真使 Mechanics Explorer 打开；启用 frame/axis 显示，截取 World、base、左右 hip 三者同屏图。请用箭头或文字标出你观察到的“前、上、右”。
6. 不要保存 `source.slx`。完成后运行 `close_system('source',0)`。

建议文件名：

```text
simulink_01_world_to_6dof.png
simulink_02_right_hip_plus_z.png
simulink_03_left_hip_minus_z.png
simulink_04_mechanics_explorer_frames.png
```

## B. MuJoCo viewer

启动：

```powershell
conda run --no-capture-output -n mujoco `
  python -m mujoco.viewer `
  --mjcf simulation/mujoco/model/scence.xml
```

在 viewer 中显示 body/site/joint frames 或 axes，完成以下截图：

1. global axes 与 `base_body/base_frame` 同屏；标出 native +X、+Y、+Z 对应机器人前/左/上中的哪一个。
2. 左右 thigh、calf、wheel 同屏，确认 +Y 位于机器人左侧。
3. 左右 wheel joint axis 同屏，确认两个 joint 的 +axis 都朝 native +Y。
4. 从前、左、上三个视角各截一张，避免仅凭单一投影视角判断正负。

建议文件名：

```text
mujoco_01_base_native_axes.png
mujoco_02_left_right_frames.png
mujoco_03_wheel_joint_axes.png
mujoco_04_front_view.png
mujoco_05_left_view.png
mujoco_06_top_view.png
```

注意：当前 compiled gravity 是零且 base 被 weld；这不影响静态 frame 截图，但不能把当前 viewer 中的动态行为当作 floating-base/IMU PASS。

## C. 真机关节正方向（低风险）

仅在现有安全调试流程、机械支撑、急停可用、驱动器电流/速度/行程限制已经生效时执行。不要使用本 Phase 临时猜测新的 torque 数值。

逐个检查 6 个驱动关节；每次只使能一个关节：

1. 记录静止时 `q_raw`、`dq_raw`。
2. 手动缓慢朝一个明确物理方向移动，确认 `q_raw` 增加还是减小。
3. 若已有经过安全批准的低速位置/速度 jog，给极小正向命令，确认 `dq_raw` 与实际运动方向；没有现成安全 jog 就跳过，不临时做 torque 注入。
4. wheel 额外记录：`q/dq > 0` 时，对应机身预期前滚还是后滚。
5. IMU 静止时记录 quaternion、gyro、accelerometer；再分别让机身做小幅正 roll、正 pitch、左转，记录哪个轴增加。

填写：

| Joint/sensor | `q>0` 或轴正向的实际运动 | `dq>0` 是否同向 | MuJoCo 当前候选 | Controller 当前语义   | PASS/不确定 |
| ------------ | -------------------------- | ----------------- | --------------- | --------------------- | ----------- |
| left hip     |                            |                   |                 |                       |             |
| left knee    |                            |                   |                 |                       |             |
| left wheel   |                            |                   | 正 q 候选为前滚 | 前滚/后滚待映射       |             |
| right hip    |                            |                   |                 |                       |             |
| right knee   |                            |                   |                 |                       |             |
| right wheel  |                            |                   | 正 q 候选为前滚 | 前滚/后滚待映射       |             |
| IMU roll     |                            |                   |                 | 正 roll 绕`{S}` +X  |             |
| IMU pitch    |                            |                   |                 | 正 pitch 绕`{S}` +Z |             |
| IMU yaw      |                            |                   |                 | 正 yaw 为左转         |             |

## D. 交付给 Codex

把截图和填写后的表放到：

```text
docs/workflow/phases/02-coordinate-interface-contract/evidence/manual/
```

如果不希望把图片提交到仓库，也可以告诉我它们的本地绝对路径。Codex 会读取这些真实产物，更新 DG05/DG06/DG07；仅口头说“看起来一致”不会标为 PASS。
