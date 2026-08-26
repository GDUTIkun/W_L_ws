# Phase 19 reuse contract

后续 SolidWorks revision 或 identified profile 不覆盖本 Phase。每次复现使用新的模型 ID 和输出目录，依次执行：

1. 保存新的 source model 与 SHA-256；运行 `build_mujoco_planar_model.py --source ... --output-dir ...`，structural diff 白名单外任何变化都阻塞。
2. 对新的 derived scene 运行 `solve_mujoco_planar_equilibrium.py`；不得复用本 Phase 的 q/reference/support。
3. 运行 `run_mujoco_planar_prefreeze_v3.py --scene ... --equilibrium ... --output-dir ...`，重新生成 A/B、gain 证据和 nonlinear envelope。
4. 从新 equilibrium 按 Adapter canonical sign/order 生成新的 formal JSON profile；不得直接复制本 Phase 的 support/reference/gain。
5. 使用同一个 C++ `ControllerCore`、`planar_standing_loop`、CSV schema 和 formal wrapper 进入新目录；阈值改变必须形成显式新 profile/decision，不修改旧 evidence。

2026-08-26 dry-run 已在 `evidence/automated/2026-08-26-regression/revision-dry-run/` 用 fresh namespace 贯通 generator→equilibrium→prefreeze。它证明入口可重跑，不等于尚未提供的新 CAD revision 已通过。
