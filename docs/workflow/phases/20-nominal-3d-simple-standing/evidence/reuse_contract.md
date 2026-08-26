# Phase 20 reuse contract

新的 CAD、质量/惯量、contact、solver 或 identified profile 不继承本 Phase 的 equilibrium、roll direction、gain、扰动幅值或PASS。每个 revision 使用新模型ID与新输出目录，顺序执行：

1. 记录 authoritative source scene/model/mesh 与 SHA-256，验证完整 freejoint、闭链、actuator、contact、solver和`2 ms` timestep不变量。
2. 运行 `solve_mujoco_3d_standing_equilibrium.py` 重新求 zero-wheel-torque upright equilibrium；不复制本 Phase reference/support。
3. 运行 `validate_mujoco_3d_standing_contract.py` 重新确认orientation Log、canonical/native sign、三路input rank/condition并生成新的`s_roll`。
4. 运行 `run_mujoco_3d_standing_prefreeze.py` 重新生成10 ms local model、gain、独立holdout与nonlinear envelope；失败不得进入Core/formal。
5. 从新的输出生成显式新 formal profile，使用同一 canonical `ControllerCore`、Adapter、`standing_3d_loop` CSV schema和wrapper写入新evidence目录。threshold变化必须形成新decision，不修改历史evidence。

2026-08-26 已在 `data/experiments/2026-08-26-phase20-revision-dry-run/` 使用fresh namespace贯通 equilibrium→contract→prefreeze，三层summary均PASS。该dry-run只证明入口可重跑，不代表尚未提供的新revision通过。
