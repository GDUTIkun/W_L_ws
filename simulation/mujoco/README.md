# simulation/mujoco

本目录保存 MuJoCo 3.7.0 模型、正式验证 fixture 和版本化仿真配置。

- `model/scence.xml`：带地面的主场景，文件名保留历史拼写。
- `model/wheel_leg.xml`：完整 imported 多刚体模型。
- `model/phase14_contact_free.xml`：完整双腿、无地面/无 contact 的约束验证 fixture。
- `model/phase14_single_leg.xml`：固定基座、无 contact 的五刚体闭链单腿 fixture；含三路 actuator、两路被动关节和三维独立运动子空间。
- `model/phase16_contact_free.xml`：保留 nominal 双腿闭链、`base_weld` 和 Adapter 命名对象，但全局关闭 contact 的确定性闭环 fixture。
- `model/phase18_wheel_contact_probe.xml`：复用左右真实 wheel mesh 的受限 carriage contact probe。
- `model/phase18_floating_contact.xml`：wheel-only collision 的完整 nominal floating-base touchdown scene。
- `config/phase14_validation.json`：Phase 14 的采样、激励、seed 和冻结阈值。
- `config/phase15_nominal.json`：Phase 15 nominal profile 的左右闭链几何、接触点、工作域、solver、有限差分和冻结阈值。
- `config/phase16_nominal.json`：Phase 16 的 2 ms physics、10 ms control、5-step ZOH、episode、fault schedule 和阈值。
- `config/phase18_nominal.json`：Phase 18 solver/contact、probe/floating case matrix 和冻结阈值。

正式内部动力学验证入口：

```bash
cd /home/t/W_L_ws
./.venv/bin/python tools/experiments/run_mujoco_internal_dynamics.py
```

该验证只支持“MuJoCo 内部自洽”结论；参数与真机一致性必须由后续共同辨识 Phase 关闭。

完整闭链运动学与 reduced Jacobian 验证入口：

```bash
cd /home/t/W_L_ws
./.venv/bin/python tools/experiments/run_mujoco_closed_chain_kinematics.py
```

该入口拒绝覆盖非空结果目录。未来模型 revision/identified profile 应使用新 config 和 run ID，不替换 nominal evidence。

Controller ↔ MuJoCo 确定性闭环入口：

```bash
cd /home/t/W_L_ws
./.venv/bin/python tools/experiments/run_mujoco_controller_loop.py \
  --output-dir data/experiments/<new-phase16-run-id>/raw
```

此入口直接运行 C++ Controller Core/Adapter/MuJoCo 循环；Python 只负责编排、校验、manifest 和比较。当前 Core 固定输出零力矩，因此本验证不代表控制效果或站立能力。

轮地接触与零控制 floating-base 验证入口：

```bash
cd /home/t/W_L_ws
./.venv/bin/python tools/experiments/run_mujoco_contact_floating_base.py \
  --output-dir data/experiments/<new-phase18-run-id>/raw
```

该入口逐 2 ms 记录接触瞬态，只支持 current profile 的 MuJoCo 内部结论；不验证站立或真实轮胎/地面参数。
