# 2026-08-25 MuJoCo Internal Dynamics

- 目的：执行 Phase 14 MuJoCo-only 内部动力学正式验证。
- 方法：`docs/experiments/mujoco_internal_dynamics_validation.md`。
- Phase：`docs/workflow/phases/14-mujoco-internal-dynamics-validation/PLAN.md`。
- 运行环境：MuJoCo 3.7.0、Python 3.12、NumPy，仓库工作树 2026-08-25 状态。
- 执行命令：`./.venv/bin/python tools/experiments/run_mujoco_internal_dynamics.py --output-dir data/experiments/2026-08-25-mujoco-internal-dynamics/raw`。
- 输入配置：`simulation/mujoco/config/phase14_validation.json`，seed `1404`，SI 单位，关节顺序为 left hip/knee/wheel 后 right hip/knee/wheel。
- `raw/`：脚本原始 JSON manifest、结果和逐样本 CSV；按仓库规则不纳入 Git。
- 受审查的同内容跟踪副本：`docs/workflow/phases/14-mujoco-internal-dynamics-validation/evidence/automated/`。
- 结果：全部九项 gate PASS；结论仅为 MuJoCo 内部自洽，不包含真机准确性。
