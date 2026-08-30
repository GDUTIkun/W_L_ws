# Phase45 Compatible-H0 Continuation

状态：`review`  
日期：2026-08-30

## 冻结 continuation

不建立新 Phase。将 `equilibrium-compatibility-formal-v2` 的唯一 compatible wrench delta
冻结为 H0 equilibrium reference：

- left/right `Fx = -0.06319271068216474 / -0.187485780522428 N`；
- left/right `Ty = +0.0025935527300793705 / +0.0027966651369644964 Nm`。

runner 启动时读取 formal-v2 authority 并以 `1e-15` tolerance 校验这四个值；不再求解或 tuning。
controller structure、unified task、3.5 Hz gain、weight、plant、contact、friction、solver 与 initial state
全部继承 Phase45 v1。planner、Phase46 tracking、12D NMPC 均未运行。

gate 顺序固定为 `EQ -> AUTH -> REAL -> SHORT -> 10 s -> post-repair reaudit`，任一 mandatory
gate FAIL 后只写 not-entered 占位证据，不执行后续层。

## Formal 结果

### DG45-EQ — PASS

compatible H0 的 actual `[ddxi_L, ddxi_R, a_t_L, a_t_R]` 为
`[4.06e-14, 2.87e-14, 1.04e-16, -6.94e-17] m/s2`。双侧 rolling active、normal load
为 `30.97/31.50 N`。因此上层 equilibrium wrench 修正确实关闭了原 tick0 static residual。

### DG45-AUTH — FAIL / mandatory stop

| mode | scale | G_QP projected | G_MJ projected | sign |
| --- | ---: | ---: | ---: | --- |
| common | 1.0 | +0.998203 | -1.875899 | mismatch |
| common | 0.5 | +0.998203 | -1.875899 | mismatch |
| common | 0.25 | +0.998203 | -1.875899 | mismatch |
| differential | 1.0 | +0.995035 | +0.237023 | match |

common actual authority 与 QP 反号；0.5/0.25 的最大 convergence error 只有
`1.43e-11`，故不是 finite-difference scale artifact。differential channel 符号和最小幅值门通过，
但 unified AUTH 要求所有 mandatory channel 通过，因此 `DG45-AUTH=FAIL`。

按 stop order，`DG45-REAL`、SHORT、10 s nominal、CONTACT/FULLBODY/WR/WBC 与
post-repair reaudit 均未进入。不能从未运行层推断 PASS/FAIL。

## Verdict

核心问题答案为 **否**：compatible equilibrium wrench 能修复静态 H0 compatibility，但不能让原
Phase45 contact-consistent rolling repair 通过完整 H0 validation。可信且收敛的 common-channel
QP-to-plant authority sign reversal 仍是 blocking structure finding。

正式 authority：`evidence/automated/compatible-h0-continuation-formal-v1/`；fresh replay：
`compatible-h0-continuation-replay-v1/`，semantic error=0。Phase45 保持 REVIEW=`REWORK`，
不创建 RECORD，不进入下一 Phase。
