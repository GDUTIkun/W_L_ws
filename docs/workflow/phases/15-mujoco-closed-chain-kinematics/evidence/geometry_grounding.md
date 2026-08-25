# Phase 15 Geometry Grounding

## 冻结对象

- Plant：`phase14_contact_free.xml`，包含 `wheel_leg.xml` 的 current nominal 双腿模型。
- 每侧 5 个 hinge：hip/knee/wheel 驱动，connect1/connect2 被动；所有局部轴均为 `+Z`，编译到 nominal 世界系后轮轴为 `+Y`。
- Closure：`connect2_site` 到 `calf_site` 的 connect equality；三行位置残差的数值秩为 2。
- 固定变换、joint/qpos/dof 地址、site 和 mesh 数据见正式 `geometry_manifest.json`。

## 轮心和名义接触点

轮心定义为 wheel body/joint 原点。左右 collision mesh 在轮轴垂直平面的编译径向最大值分别约为 `0.05012073 m` 和 `0.05012075 m`，径向直径约 `0.10 m`，轴向宽度约 `0.04 m`。

Phase 15 将可微名义圆半径冻结为 `0.05 m`，接触点局部坐标冻结为 `[0.05, 0, 0] m`。最大 mesh/名义半径差为 `1.2075152764200875e-4 m`，低于预冻结 `2e-4 m` gate。

这只是 current nominal MuJoCo profile。它不是瞬时 contact manifold，也不是轮胎变形模型或真机标定。Simulink 的 `0.08 m` 半径继续作为其自身简化假设，不进入 Phase 15 MuJoCo contact-point profile。

## 方向和左右契约

- 零位轮轴：世界 `+Y`。
- 正 wheel 角速度：底部材料点速度 `-X`；无滑约束下对应轮心滚动方向 `+X`。
- 左右轮心和名义接触点关于世界 XZ 面镜像。
- 左右 wheel body frame 使用相同右手轴定义；不把 determinant 为 `-1` 的空间反射误当作 SO(3) 姿态。
- nominal 被动分支：左 `[connect1, connect2]=[knee,-knee]`；右 `[-knee,-knee]`。

