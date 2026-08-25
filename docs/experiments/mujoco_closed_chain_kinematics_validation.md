# MuJoCo 完整闭链运动学与 Jacobian 验证方法

## 目的与边界

本方法验证 current nominal MuJoCo 五刚体腿的闭链装配分支、轮心/名义接触点位姿和约束降维 Jacobian。它只形成 MuJoCo 内部几何与数值一致性证据，不使用真机数据，也不证明 collision mesh、轮半径或接触模型与真机一致。

## 固定输入

- Plant：`simulation/mujoco/model/phase14_contact_free.xml`，MuJoCo 3.7.0；关闭 contact，基座保持 nominal `qpos0`。
- Profile/config：`simulation/mujoco/config/phase15_nominal.json`。
- 独立坐标：每侧 `[hip, knee, wheel]`；被动坐标：`[connect1, connect2]`。地址全部由命名对象解析，不依赖 XML 顺序。
- 闭链约束：`connect2_site - calf_site = 0`。三维位置残差的有效秩为 2，与平面闭链的两路被动坐标一致。
- 名义接触点：轮体局部 `[0.05, 0, 0] m`。`0.05 m` 来自当前编译 collision mesh 约 `0.10 m` 的径向直径，和 Simulink 的 `0.08 m` 简化假设明确分离。

## 被动关节与装配分支

给定独立坐标后，runner 使用 closure site 残差和 MuJoCo site Jacobian 的被动列执行确定性 Newton/least-squares 求解。零位作为 nominal 分支起点，knee 正负方向分别连续延拓；随后反向回放同一路径检查分支跳变。

独立几何参考给出 nominal 分支关系：

- 左腿：`connect1 = knee`、`connect2 = -knee`。
- 右腿：`connect1 = -knee`、`connect2 = -knee`。

求解器结果必须同时满足 closure residual、上述独立关系、正反路径一致性和预冻结迭代上限。

## FK 与完整 Jacobian

Profile 保存左右支链的固定平移和固定旋转。runner 不读取 MuJoCo runtime pose 来构造参考，而是独立递推：

```text
T_child = T_parent · Trans(p_fixed) · R_fixed · Rz(q)
```

由此分别得到 hip→calf→wheel 支链、hip→connect1→connect2 支链、两个 closure site、轮心、轮体姿态和名义接触点。完整点 Jacobian 使用世界坐标中的转轴/关节原点：

```text
Jv_i = axis_i × (point - origin_i)
Jw_i = axis_i
```

解析结果与 `mj_jacBody`、`mj_jacSite`、`mj_jac` 对照。

## 约束降维 Jacobian

完整闭链 Jacobian 按独立/被动坐标分块：

```text
Jc = [Ja Jp]
Jp · dq_passive = -Ja · dq_active
S = [I; -pinv(Jp)·Ja]
J_reduced = J_object · S
```

每个样本记录 `Jc·S`、`Jp` 最小奇异值和条件数。解析 profile 与 MuJoCo 分别构造自己的 `Jc`、`S` 和轮心/接触点 reduced Jacobian，避免共享同一中间结果自证。

第三路参考对每个独立坐标执行中心有限差分。`q±epsilon` 每次都重新求被动关节；轮体角速度使用 `log(R_plus·R_minus^T)/(2·epsilon)`。冻结 epsilon 为 `1e-4 / 3e-5 / 1e-5 rad`，正式误差使用 `1e-5 rad`，同时保存完整 epsilon sweep。

## 速度、虚功和方向

- 使用冻结 `dq_active` 比较解析和 MuJoCo 的 `twist = J_reduced·dq_active`。
- 使用 `mj_applyFT` 独立施加冻结世界系 wrench，再比较 `S^T·qfrc` 与 `J_reduced^T·wrench`。
- 比较 `tau_active^T·dq_active` 与 `force^T·v + torque^T·omega`。
- 在零位，轮轴世界方向为 `+Y`；正 wheel 角速度使底部材料点沿 `-X`，对应无滑轮心滚动方向 `+X`。

## 工作域与通过条件

冻结网格为每侧 `5 hip × 7 knee × 3 wheel = 105` 个样本，总计 210 个。每个样本都进入 `workspace.csv`，失败、不可达或近奇异样本不得删除。正式阈值全部位于 profile/config，先于正式 run 固定。

只有以下项目全部通过才可放行：geometry/profile、装配分支、工作域、独立 FK/full Jacobian、三方 reduced Jacobian、速度/虚功、左右对称和完整重复运行确定性。

## 执行与输出

```bash
cd /home/t/W_L_ws
./.venv/bin/python tools/experiments/run_mujoco_closed_chain_kinematics.py
```

可通过 `--profile`、`--config` 和 `--output-dir` 选择未来 revision/profile。非空输出目录会直接失败，防止覆盖旧 evidence。

- `phase15_validation.json`：gate、最差指标和总结果。
- `geometry_manifest.json`：对象地址、轴、site、mesh/contact-point provenance 和模型 hash。
- `run_manifest.json`：profile、模型/config/runner hash、solver、工作域、阈值和 `supersedes`。
- `workspace.csv`：210 个样本的被动解、closure、奇异值、FK/Jacobian/finite-difference/虚功指标。

