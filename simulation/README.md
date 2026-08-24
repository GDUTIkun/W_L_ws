# simulation

## 目录职责

保存可执行仿真资产，分为已成功复现的 Simulink 基线和后续 MuJoCo 工程。技术说明与实验结论写入 `docs/`，控制器产品代码写入 `ros_ws/`。

## 预期结构

```text
simulation/
├── simulink_baseline/  # 成功复现且受控的 Simulink 对照基线
└── mujoco/     # MJCF/URDF、场景、资产和仿真配置
```

## 允许内容

- 成功复现的 Simulink 模型、数据字典和运行入口；
- MuJoCo 模型、场景、机器人资产及适配所需配置；
- 小型、可版本控制的基准输入和验证配置。

## 禁止内容

- 仿真日志、批量结果、缓存和生成代码；
- STM32 固件；
- 与 ROS/MuJoCo 强耦合的 Controller Core 副本；
- 尚未运行成功却写入 README 的启动命令。

## 上下游关系

[`simulink_baseline/`](simulink_baseline/README.md) 提供算法对照基线；MuJoCo Adapter 从 MuJoCo 生成统一 RobotState 并应用 TorqueCommand；Phase 文档记录两端一致性证据。

## 当前状态

已成功复现的 Simulink 基线存放在 `simulink_baseline/`；MuJoCo 工程尚未落地。

## 维护规则

- 基线模型变更前先在候选副本中复现验证；通过后才更新 `simulink_baseline/`。
- MuJoCo 模型参数必须标明来源，避免 CAD 惯量与执行器附加惯量重复计入。
- 运行输出写入被忽略的结果目录，不提交大型数据。
