# Phase 02 人工坐标证据审查

审查日期：2026-08-25

Verdict: `PARTIAL PASS`

本目录的证据足以关闭 Simscape/MuJoCo world 轴方向，但不足以关闭 base/IMU 原点、跨系统 joint sign 或真机安装语义。

## 逐项审查

| 证据 | 结论 | 状态 |
| --- | --- | --- |
| `word2body.png` | World Frame 经无旋转、仅 `+Y` 平移的 Rigid Transform 接到 6-DOF Joint base；与自动 manifest 一致 | PASS |
| `正视.png`、`右视.png`、`俯视.png` | Simscape 物理轴为 X 前、Y 上、Z 右，重力沿 -Y | PASS |
| Rigid Transform5 / 10 | 自动 manifest 已记录左右髋连接；用户再次确认，未重复截图可接受 | PASS |
| `mujoco正视.png`、`mujoco右视图.png`、`mujoco左视图.png` | MuJoCo world 为 X 前、Y 左、Z 上；与 XML 数值和 FK 结果一致 | PASS |
| `image.png` | 显示当前 base/imu 标注和局部轴；只能证明当前 site 与 CAD frame 的关系，不能证明其原点是 COM 或真实 IMU 安装点 | PARTIAL |
| 关节正向 | 图片不能代替 `q += epsilon` 的跨 Simulink/MuJoCo/真机运动方向核对 | OPEN |
| 真机 frame | 用户明确延后至 MuJoCo frame/site 落地后补齐 | DEFERRED |

## 技术处置

1. Canonical world 采用 FLU：X 前、Y 左、Z 上。
2. Simulink baseline 保留 X 前、Y 上、Z 右；通过 `R_N_from_S` 显式映射，不修改 baseline 脚本中的既有重排。
3. MuJoCo world 与 canonical FLU 同轴，`R_N_from_M=I`。
4. 当前 `base_body` 定义为 `base_cad_frame`；另行定义位于 torso rigid-body COM 的 `base_control_frame`，真实 `imu_frame` 独立处理。
5. MuJoCo 编译模型给出的当前 nominal torso COM 相对 CAD 原点为 `[-0.077378152, 0.000000810, -0.032277680] m`。这不是最终真机标定值。
6. 不移动 `base_body` XML 原点；后续在 Adapter/辅助 site 中表达 COM frame，避免联动修改整个导入树。

## 已解决 finding：`source.slx`

重新运行只读 inspector 时，当前工作树中的 `source.slx` 与旧 manifest 相比，`source/PD_only/6-DOF Joint` 的 LConn2–LConn7 已不再连接 `Simulink-PS Converter8–13`。右侧 Px/Vx/Py/Vy/Pz/Vz/Q/w 测量连接和坐标相关参数未发现变化。

用户已确认该断线是有意修改。权威 manifest 已按当前模型重生成，5 s closed-loop smoke 结果为 `simulationCompleted=true`、`controlStable=true`，因此该 finding 已关闭。

## 转交后续 Phase 的 gate

- Phase 04：逐 joint zero offset 的 matching-pose + FK 回归。
- Phase 06：真实 IMU 安装 frame、encoder offset 和 torque 方向复核。
- Phase 07：真实质量与 COM 标定。
