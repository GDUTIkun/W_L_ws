# simulation/mujoco

本目录保存 MuJoCo 3.7.0 模型、正式验证 fixture 和版本化仿真配置。

- `model/scence.xml`：带地面的主场景，文件名保留历史拼写。
- `model/wheel_leg.xml`：完整 imported 多刚体模型。
- `model/phase14_contact_free.xml`：完整双腿、无地面/无 contact 的约束验证 fixture。
- `model/phase14_single_leg.xml`：固定基座、无 contact 的五刚体闭链单腿 fixture；含三路 actuator、两路被动关节和三维独立运动子空间。
- `config/phase14_validation.json`：Phase 14 的采样、激励、seed 和冻结阈值。

正式内部动力学验证入口：

```bash
cd /home/t/W_L_ws
./.venv/bin/python tools/experiments/run_mujoco_internal_dynamics.py
```

该验证只支持“MuJoCo 内部自洽”结论；参数与真机一致性必须由后续共同辨识 Phase 关闭。
