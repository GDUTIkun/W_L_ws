# Phase 14 模型参数 Grounding

## 编译模型边界

- 完整 contact-free fixture 编译为 `nq=17`、`nv=16`、`nu=6`、`neq=3`，包含 `base_weld` 与左右 `connect` closure。
- 完整模型共有 11 个非 world body，总 nominal mass 为 `6.4344 kg`。
- 约束 Jacobian 在精确平面轴线修正后 rank 为 `10`，因此 constrained nullspace 为 `6` 维，与左右各 hip/knee/wheel 三个独立运动方向一致。
- 单腿 fixture 是固定基座的五刚体闭链，`nq=nv=5`、`nu=3`、`neq=1`；closure Jacobian rank 为 2，独立运动子空间为 3 维。需要隔离 actuator/inertia/energy 时显式关闭 closure，只分析 hip→knee→wheel 串联支路。

机器可读逐项数值见 [`parameter_manifest.json`](automated/parameter_manifest.json)。

## Provenance 分类

| 参数 | 当前来源 | 状态/解释限制 |
| --- | --- | --- |
| geom mass | imported MJCF 中的 nominal 数值 | 未经真机称量验证 |
| body mass/COM/principal inertia/inertial frame | MuJoCo 3.7.0 根据 mesh 与 geom mass 编译派生 | internally compiled；不是 CAD/真机标定结论 |
| joint axis/anchor/body transform | imported MJCF；Phase 14 将截断的 `pi/2`、`pi` 欧拉角改为精确值 | 坐标契约已回归；真实装配仍待验证 |
| damping/frictionloss/armature | MJCF 未显式设置，编译值为零 | 真实值 unknown，必须后续辨识 |
| actuator gear | Phase 04 unit-gear ideal torque interface | 不是电机、减速器或驱动器标定模型 |
| gravity/timestep/integrator/solver | versioned fixture/config nominal 设置 | 用于可重复仿真，不代表已校准数值精度 |
| wheel-floor contact | 本 Phase 正式动力学 fixture 禁用 | 不形成接触保真度结论 |

## Phase 04 模型 finding

原模型的 `1.5708` 和 `3.14159` 截断欧拉角使名义共轴关节偏离约 `3.67e-6`，闭链 equality Jacobian 被数值上判为 rank `11`，而不是物理预期的 `10`。Phase 14 将这些结构角改为精确 `pi/2`/`pi`，随后：

- canonical 关节轴测试继续 PASS；
- constrained nullspace 恢复为 6 维；
- joint zero offsets 按修正后编译几何重新计算；
- Phase 04 六项 Adapter 测试全部回归 PASS。

## Decision Gates

- DG01：由两个 versioned fixture 和按名字 invariant check 关闭。
- DG02：由本 grounding 与机器可读 manifest 关闭；unknown 没有被改写为 nominal。
- 真机 mass/COM/inertia、passive 参数和 actuator 参数继续转后续共同辨识，不在本 Phase 关闭。
