# Simulink–MuJoCo joint coordinate mapping

Status: `axis/sign frozen; zero offsets deferred`

## Evidence

- Simscape Revolute Joint 的 primitive 正轴是其 base frame `+Z`。
- `Rigid Transform5/10` 和腿内 `Rigid Transform1–9` 均无 rotation，所以六个 Simulink 驱动关节的正轴都是 `+S_z=-N_y`。
- MuJoCo 编译模型在 qpos0 下，六个驱动 joint world axis 都是约 `[0,+1,-3.7e-6]`，即 `+N_y`；左右 hip/knee 的 `q += epsilon` wheel-center 导数同侧同号。
- Simulink `Fcn/Fcn1` 对 hip measurement 使用 `q_controller_hip=q_simscape_hip+pi/2`；knee/wheel 不做该补偿。

## Frozen sign mapping

令 `q_M` 为 MuJoCo joint coordinate，`q_C` 为 Controller 使用的 joint coordinate。每个驱动关节都采用：

```text
q_C  = -q_M + b_joint
dq_C = -dq_M
tau_M = -tau_C
```

`tau` 的负号由功率不变性得到：`tau_C*dq_C = tau_M*dq_M`。左右两侧不增加镜像负号。

MuJoCo wheel 的 `q_M>0` 绕 `+N_y`。底部半径沿 `-N_z` 时，无滑动所需轮心速度为 `-omega×r=+N_x`，因此对应向前滚。Simulink/Controller 正 wheel rate 的原生轴相反，映射后物理 rolling 方向一致。

## Offset gate

`b_joint` 不能由 joint 名称或 CAD 零位猜测。当前 MuJoCo `q=0` 的导入装配姿态与 Simulink 简化两连杆 nominal pose 不一致，且两者几何长度/闭链表达不同。

Phase 04 必须选择一个数值可复现的 matching pose，同时记录：

1. MuJoCo 六个 `q_M`；
2. Simulink/Controller 六个 `q_C`；
3. hip、knee、wheel-center 的 canonical FLU 位置；
4. 由 `b_joint=q_C+q_M` 得到的逐关节 offset；
5. 至少第二个 pose 的 FK 回归，证明 offset 不是只对单点凑合。

offset 未冻结前，MuJoCo Adapter 不得把 raw qpos 直接发布为 Controller joint state。真实 encoder offset 和 torque 方向在 Phase 06 的低风险验证中复核。
