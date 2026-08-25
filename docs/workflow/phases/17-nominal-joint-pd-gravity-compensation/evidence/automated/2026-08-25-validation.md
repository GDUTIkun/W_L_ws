# Phase 17 自动验证记录（2026-08-25）

## Formal-v3

执行 `run_mujoco_joint_pd_gravity.py` 到新目录 `2026-08-25-formal-v3/`，结果 `overall_pass=true`，14/14 gate PASS。原 `formal/` 与 `formal-v2/` 保留未覆盖；v2 新增 C++ diagnostics 与 JSON gravity profile 的逐 tick 交叉检查，v3 再把 gravity offset/coefficients 从 versioned JSON 注入 runner，证明模型 revision 可换 profile 而不用改控制算法。

- gravity：150 姿态；reduced-bias 最差误差 `5.266e-12 N·m`，势能梯度最差误差 `6.782e-9 N·m`，C++/profile 误差 `0`；wheel 偏心项最大 `4.066e-4 N·m`。
- hold：PD+gravity 最终最大误差 `0.002831 rad`，PD-only 为 `0.115862 rad`；所有数据有限，ZOH 差为 `0`。
- steps：六关节、正负方向全部 PASS；hip/knee settling `0.30–0.46 s`，wheel `0.88–0.89 s`；峰值速度分别不超过 `0.631`、`0.610`、`1.804 rad/s`。
- symmetry：三类最差左右差 `8.14e-5 rad`；三类 disturbance 最终最大恢复误差 `0.002832 rad`。
- saturation、episode reset replay、fresh-process bitwise replay 全 PASS。

## Build/tests 与历史回归

- Jazzy build：`wheel_leg_core`、`wheel_leg_msgs`、`wheel_leg_ros`、`wheel_leg_mujoco` PASS。
- `colcon test-result --verbose`：19 tests，0 errors/failures/skipped；包含默认 zero、非法/合法 ROS profile、Core PD/gravity/clamp/reset 与 Adapter tests。
- coordinate contract PASS。
- Phase 14 九类内部动力学回归 PASS；Phase 15 八类闭链运动学/Jacobian 回归 PASS。
- Phase 16 扩展 runner 上 24/24 zero/fault/determinism gates PASS；历史正式 evidence 未覆盖。

本记录只支持 current nominal、fixed-base、contact-disabled simulation-only 结论，没有使用真机数据。
