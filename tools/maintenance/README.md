# tools/maintenance

## 目录职责

保存仓库维护脚本，例如链接检查、格式检查、数据格式转换、CBM/Graphify 索引维护入口。

## 维护规则

- 明确说明是否会修改工作树、索引或外部系统。
- 不把实验业务逻辑放入本目录。
- Graphify/CBM 更新是独立维护动作，不隐式夹带在实验或产品运行脚本中。

## Phase 02 坐标维护入口

- `inspect_simulink_frames.m`：只读提取 Simulink/Simscape frame、端口和测量参数。
- `audit_mujoco_frames.ps1`：静态解析 MJCF body/site/joint/sensor/equality。
- `audit_mujoco_runtime.py`：在 `conda:mujoco` 中编译模型并生成运行时 frame/sensor/COM 证据。
- `test_coordinate_frame_contract.m`：验证 Simscape→FLU、legacy field pack、yaw 和 wrench 变换。
- `test_mujoco_coordinate_contract.py`：验证 MuJoCo FLU、COM site、joint sign、rolling 和 quaternion 契约。
