# Phase 19: exact 2D sagittal 简单站立 — RECORD

Status: `complete`

## Outcome

current nominal MuJoCo 已具备从 authoritative 3D CAD model 可重复派生的 exact sagittal plant，并在 canonical C++ Controller Core 上完成 fixed-leg/simple-standing 控制。正式 10 s case matrix、fault/reset、replay、non-overwrite 与历史回归全部 PASS；结论严格限制为 simulation-only exact 2D。

## Delivered

- source→planar generator、compiled structural diff 与 fresh revision-workflow 入口。
- zero-wheel-torque contact/equality equilibrium solver和 canonical state/sign contract。
- v3 sampled-leg attribution与 pre-freeze runner：standing-specific leg `Kp=8, Kd=1`，common-wheel gain `[2.93971694, 5.56292601, 39.49917236, 0.62964133]`。
- Controller Core `kSimpleStanding` mode、support/PD/common-wheel diagnostics、strict bilateral contact 和 fail-closed latch/reset。
- C++ `planar_standing_loop`：2 ms physics、10 ms control、5-step ZOH、contact-projected reset、disturbance/fault injection 和 CSV logging。
- versioned formal profile/wrapper、primary/replay manifests、full source/binary/config hashes、reuse contract 与 Graphify maintenance prompt。

## Key Results

- formal-v4：11/11 normal/perturbation cases PASS，4/4 fault cases PASS。
- max pitch `0.005 rad`、max X error `0.004176 m`、max height error `0.000409 m`、max active-leg error `0.012130 rad`。
- bilateral contact fraction `1.0`；equal-wheel torque、Adapter sign 与 5-step ZOH errors 全部精确为 `0`。
- primary/replay summary SHA-256 均为 `adb1fd39d1ce21e47da49cf1352170d2e03190f103ae865d42a6a0ab25a4f373`。
- ROS/C++ 19 tests 无失败；Phase 02/14/15/16/17/18 regressions 全 PASS。

## Decisions

- timing 继续冻结为 `0.002 s / 0.010 s / 5-step ZOH`；不引入隐藏的 2 ms controller inner loop。
- Phase 17 fixed-base gain `12/1.5` 不被覆盖；Phase 19 contact-standing profile 独立使用 `8/1`。
- Core 首个 standing sample 就要求双轮接触；非接触 reset 的处理属于 runner 初始化投影，不放宽 runtime safety。
- formal pitch initial envelope 为 contact-consistent `±0.005 rad`；更大或不在 constraint manifold 的坐标 reset 不形成 PASS claim。
- raw full-coordinate finite-difference poles 只作 nonsmooth/constraint diagnostic；release 由 admissible local convergence 和 full nonlinear formal 决定。
- `formal-v4` 是唯一最终 authority；此前 formal、v2、v3 与两轮 REWORK review 永久保留为非覆盖历史。

## Evidence

- [Formal summary](evidence/automated/2026-08-26-formal-v4/summary.json)
- [Formal manifest](evidence/automated/2026-08-26-formal-v4/manifest.json)
- [Automated validation](evidence/automated/2026-08-26-validation.md)
- [Pre-freeze validation](evidence/exploratory/2026-08-26-planar-prefreeze-v3/validation.md)
- [Reuse contract](evidence/reuse_contract.md)
- [Review](REVIEW.md)

## Reproduction

```bash
cd /home/t/W_L_ws/ros_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to wheel_leg_mujoco
cd ..
./.venv/bin/python tools/experiments/run_mujoco_planar_standing_formal.py \
  --output-dir data/experiments/<new-phase19-run-id>
```

输出目录必须为空。新 CAD/identified revision 必须先按 [reuse contract](evidence/reuse_contract.md) 重新派生 plant、求 equilibrium、生成 gain/profile，不得直接复用本次数值或覆盖本次 evidence。

## Limits and Next Use

- 不证明完整 3D、真机、roll/yaw/turning、单轮支撑、坡地或大扰动恢复。
- contact/friction/compliance 仍是 nominal MuJoCo 参数，未经过 MuJoCo–real identification。
- 下一 Phase 可开始 nominal 完整 3D 简单站立；至少显式加入 lateral/roll/yaw state、actuation/control authority 和独立 formal matrix。
