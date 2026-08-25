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
