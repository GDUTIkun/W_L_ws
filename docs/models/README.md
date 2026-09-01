# docs/models

## 目录职责

保存模型本身的技术说明：它代表什么、状态与输入如何定义、参数来自哪里、在哪些条件下有效。这里不存放模型文件或实验数据。

## 适用内容

- 坐标系、单位、关节顺序和符号约定；
- 机械、执行器、传感器、接触和控制模型的状态/输入/输出定义；
- MuJoCo 参数、CAD 参数、辨识参数的来源和适用范围；
- Simulink baseline、C++ Controller 和 MuJoCo 模型之间的语义映射。

## 不适用内容

- `.slx`、MJCF、URDF 或产品源码；这些放入 `simulation/` 或 `ros_ws/`。
- 实验方法与单次实验结果；分别放入 `docs/experiments/` 和 Phase RECORD。
- 大型数据、图和运行日志。

## 命名建议

使用主题名称，例如 `coordinate-conventions.md`、`actuator-model.md`、`mujoco-parameter-provenance.md`。一个文档只描述一个稳定模型契约。

## 当前模型文档

- [当前 MuJoCo 轮腿模型（Model B）](current_mujoco_model.md)：当前 plant、闭链拓扑、canonical Adapter 边界与 12/16 维动力学映射。
- [Simulink MPC–WM-WBC baseline](simulink_mpc_wm_wbc_baseline.md)：三维 Simscape plant、16-state NMPC、12-DoF weighted WM-WBC、slack 契约、采样链、平地证据和 terrain failure 边界。

## 维护规则

- 参数必须附来源：CAD、器件手册、静态标定、辨识结果或假设。
- 模型变更必须说明对 Simulink reference、MuJoCo 和 Controller 的影响。
- 未经真实证据支持的参数标记为候选，不写成已验证结论。
