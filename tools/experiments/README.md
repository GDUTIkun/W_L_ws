# tools/experiments

## 目录职责

保存正式设计实验的执行、采集、激励和回放脚本。

## 何时使用

仅当实验需要可复现、会产生可审查数据，或其结果将影响技术决策/Phase 放行时使用。脚本应对应 `docs/experiments/` 中的方法，并把输出写入 `data/experiments/<run-id>/`。

## 脚本最小说明

每个脚本或子目录 README 说明：对象、依赖、输入、设备/仿真版本、输出目录、运行命令和失败时的安全行为。

## 不适用内容

- 临时画图、一次性小验证和探索性测试；使用 `tools/scratch/`。
- 离线数据拟合、统计与绘图主逻辑；使用 `tools/analysis/`。
- 产品运行时代码。

## 当前入口

- `run_mujoco_internal_dynamics.py`：执行 Phase 14 的 MuJoCo-only FK/Jacobian、重力、质量矩阵、正逆动力学、闭链、耦合、能量和确定性回放验证。方法见 `docs/experiments/mujoco_internal_dynamics_validation.md`，默认输出到 Phase 14 evidence。
- `run_mujoco_closed_chain_kinematics.py`：执行 Phase 15 的 profile-driven 闭链被动解、独立 FK、约束降维 Jacobian、有限差分、速度、虚功、工作域与非覆盖验证。方法见 `docs/experiments/mujoco_closed_chain_kinematics_validation.md`。
- `run_mujoco_controller_loop.py`：编排 Phase 16 的 C++ Controller↔Adapter↔MuJoCo 固定步数循环，校验 5-step ZOH、双时钟、reset/fail-safe、fresh/reset replay，并生成逐 tick CSV、SHA-256 manifest 和汇总。方法见 `docs/experiments/mujoco_controller_loop_validation.md`。
- `run_mujoco_joint_pd_gravity.py`：编排 Phase 17 的解析重力双 oracle、Joint PD+gravity hold/阶跃/对称/扰动/饱和/replay 正式矩阵，并生成非覆盖逐 tick evidence 与 manifest。方法见 `docs/experiments/mujoco_joint_pd_gravity_validation.md`。
- `run_mujoco_contact_floating_base.py`：执行 Phase 18 的 actual-wheel probe 与完整机器人零控制 floating-base 验证，覆盖 wheel-only collision、normal/rolling/lateral/friction、触地、base state、闭链和 reset replay。方法见 `docs/experiments/mujoco_contact_floating_base_validation.md`。
- `build_mujoco_planar_model.py`：从 authoritative `wheel_leg.xml` 非覆盖派生 Phase 19 exact `X/Z/pitch` plant，并审计除 base DOF topology 外的 XML/compiled physics 差异；输出 derived model、scene、audit 和 hash manifest。
- `solve_mujoco_planar_equilibrium.py`：求解 Phase 19 柔性闭链/接触一致、左右轮力矩严格为零的 exact-planar 静态平衡点，并输出可重放审计证据。
- `validate_mujoco_planar_state_contract.py`：验证 Phase 19 `x/dx/pitch/pitch-rate` site/Jacobian/有限差分符号，以及 native/canonical wheel rolling 与 Adapter 符号关系。
- `run_mujoco_planar_prefreeze.py`：同时比较 Phase 19 四状态 reset-local 模型与完整 26 状态 sampled plant，并用非线性 holdout 决定是否允许进入 Core。
- `view_mujoco_planar_standing.py`：加载 Phase 19 冻结的 exact-planar scene/profile，在 MuJoCo viewer 中按 `2 ms / 10 ms / 5-step ZOH` 播放站立闭环；仅供动画观察，不替代 C++ formal evidence。
