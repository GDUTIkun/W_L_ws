# Phase 43 Candidate Definition

正式 config：`simulation/mujoco/config/phase43_rolling_repair_v1.json`。

| Candidate | Frozen change | Independent task rows | Gains |
| --- | --- | --- | --- |
| R43-A | original interaction wrench 的 left/right `Fx`、`Ty` 四维 deterministic trim | 0 | none |
| R43-B | canonical reduced wheel joint acceleration rows 8/11，target `-Kd*qdot_w` | 2 | `Kd=2*pi*f` |
| R43-C | existing affine wheel-origin longitudinal acceleration rows，`xi_ref=xi(t0)` | 2 | `Kp=(2*pi*f)^2`, `Kd=2*(2*pi*f)` |
| R43-D | R43-C + R43-B | 4 | 上述两组 |

三档 `f={2.5,3.5,5.0} Hz`，native task normalization固定20 rad/s²。component test证明
D 的 Hessian/gradient增量严格等于 B+C，B 与 C 不重叠。canonical/native joint sign相反，但
`qdd_des=-Kd*qdot` 经两次符号映射保持不变；报告仍使用 raw native与`v_r=-0.05*qdot_native`。

A 的 formal trim为：

```text
Delta[left Fx, right Fx, left Ty, right Ty]
= [-0.260332461, -0.265906154, -0.071362307, -0.070729473]
```

solver `nfev=48 <= 60`；144 是包含四维数值 Jacobian probes 的实际函数调用计数，不是 SciPy
`max_nfev`口径。A 没有 feedback，未修改 Model B 或 contact。

审计确认 Phase34 run=false、12D NMPC run=false、16D repair=false、plant/contact modification=false。
