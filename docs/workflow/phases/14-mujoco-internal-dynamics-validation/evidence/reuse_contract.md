# Phase 14 后续共同辨识复用契约

## 输入

- MuJoCo 版本、scene、seed、采样姿态、激励、solver/timestep 和阈值全部来自 `simulation/mujoco/config/phase14_validation.json`。
- `ctrl_native_*_nm` 使用 MuJoCo 原生关节正方向；进入 canonical Controller/Hardware 边界时必须应用 `tau_M=-tau_C`。
- Canonical 关节顺序固定为 left hip/knee/wheel、right hip/knee/wheel；位置/速度映射保持 `q_C=-q_M+b`、`dq_C=-dq_M`。

## 日志 schema

`open_loop_replay.csv` 每行包含：

1. `time_s`；
2. 三路单腿 `q_*_rad`、`dq_*_rad_s`、`qdd_*_rad_s2`；
3. 三路 `ctrl_native_*_nm`；
4. 最大约束位置/速度残差；
5. `mechanical_energy_j`。

真机比较不得把 motor current、controller canonical torque 或未经校准的估算力矩直接写入 `ctrl_native`；应保留原始字段并通过显式映射生成可比较列。

## 复现与比较入口

```bash
./.venv/bin/python tools/experiments/run_mujoco_internal_dynamics.py
```

Phase 05/07/08 可复用配置中的姿态、seed、激励频率、SI 单位、关节顺序和 CSV 字段，但必须另建真机安全限制、时间同步、传感器质量和 torque calibration gate。Phase 14 baseline 不能替代这些证据。
