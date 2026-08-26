# Phase 20: nominal 完整 3D 简单站立 — RECORD

Status: `complete`

## Outcome

current nominal完整3D MuJoCo plant已在canonical C++ Controller Core上完成simulation-only简单站立。控制器以common wheel稳定X/pitch、差分腿力矩稳定roll、differential wheel保持reset heading；正式10 s case matrix、plant/contact指标、fault/reset、replay、non-overwrite、历史回归与reuse入口全部PASS。

## Delivered

- full-3D zero-wheel-torque upright equilibrium solver、fresh replay与compiled invariant evidence。
- quaternion Log/state/sign contract、三路virtual input oracle与冻结单位`s_roll`。
- 10 ms contact-mode中心差分模型、static LQR gain和独立nonlinear tuning/holdout pre-freeze。
- additive Controller Core `kSimpleStanding3d`、x8/u3 diagnostics、support+PD+balance decomposition和fail-closed latch/reset。
- 独立C++ `standing_3d_loop`，提供full-3D reset/disturbance/fault、2 ms/10 ms/5-step ZOH、control与plant CSV日志。
- versioned formal-v3 profile/wrapper、primary/replay manifest/hash、验证方法和revision reuse contract。

## Key Results

- equilibrium：max qacc `2.45e-11`、generalized residual `1.51e-10`、closure `1.63e-4 m`、左右wheel load `30.96/32.16 N`、one-step qvel drift `4.90e-14`。
- pre-freeze：8-state controllability rank `8`，training/validation RMS `0.0437/0.0396`，frozen closed-loop spectral radius `0.9910`。
- formal-v3：19/19 normal/perturbation、6/6 fault cases PASS；bilateral contact fraction `1.0`。
- worst state：`|x|=0.00176 m`、`|y-y0|=0.00153 m`、height error`0.000283 m`、pitch/roll/yaw=`0.00531/0.00476/0.00448 rad`。
- worst plant：minimum wheel load`30.17 N`、penetration`0.000525 m`、rolling/lateral slip`0.00954/0.00157 m/s`、closure`0.000185 m`。
- ZOH、Adapter sign与virtual mapping error均为`0`；primary/replay 26个文件exact，summary SHA-256为`fc3322f7f684240857003f4de9fee396764fdedd543d723de9293efdfd7aecc3`。
- ROS/C++ 19 tests、coordinate contract、Phase18 plant、Phase19 planar formal与fresh reuse pipeline全部PASS。

## Decisions

- 保持authoritative Phase18 full-3D plant与`0.002 s / 0.010 s / 5-step ZOH`，不派生planar/hidden-constrained release model。
- runtime state固定为`[x-x0,vx,pitch,wy,roll,wx,yaw-yaw0,wz]`；orientation使用world-axis shortest-arc Log。
- `u=-Kx`为无积分静态反馈；runtime Core不链接MuJoCo。Y/Z只作为真实plant outcome和safety gate。
- raw roll reset离开bilateral-contact mode，不进入formal；roll正负方向由world-X moment验证。
- formal-v3是唯一最终authority；formal-v1/v2及失败的exploratory辨识/LQR尝试永久保留且不覆盖。
- 新CAD/contact/identified revision必须按reuse contract重新求equilibrium、`s_roll`、model、gain和formal profile。

## Evidence

- [Formal summary](evidence/automated/2026-08-26-formal-v3/summary.json)
- [Formal manifest](evidence/automated/2026-08-26-formal-v3/manifest.json)
- [Fresh replay summary](evidence/automated/2026-08-26-formal-v3-replay/summary.json)
- [Validation method](../../../experiments/mujoco_3d_simple_standing_validation.md)
- [Reuse contract](evidence/reuse_contract.md)
- [Review](REVIEW.md)

## Reproduction

```bash
cd /home/t/W_L_ws/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to wheel_leg_mujoco
cd ..
./.venv/bin/python tools/experiments/run_mujoco_3d_standing_formal.py \
  --output-dir data/experiments/<new-phase20-run-id>
```

输出目录必须为空。新的model/profile先按[reuse contract](evidence/reuse_contract.md)重跑pre-freeze，不得复制本Phase数值或覆盖正式evidence。

## Limits and Next Use

- 不证明真机、identified plant、WBC/QP/NMPC、turning、absolute-Y regulation、单轮支撑、坡地、大扰动或跌倒恢复。
- contact/friction、质量/惯量和执行器仍是current nominal MuJoCo参数，尚未与真机辨识。
- 下一Phase可建立nominal Weighted WBC，但必须新增独立PLAN/decision gates；不能把本Phase的静态gain直接扩张为WBC结论。
