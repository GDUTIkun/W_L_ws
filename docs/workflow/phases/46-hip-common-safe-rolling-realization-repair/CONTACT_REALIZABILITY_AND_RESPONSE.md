# Contact realizability and response

左右轮 actual two-point、每点 3D force 的 `G_p` 均为 `6x6 rank 5`；非零 singular-value
condition number 约 `50`，缺失 wrench direction 几乎纯 lateral-axis moment `Ml`。
nominal compatible wrench 的不可实现 fraction 仅 `8.73e-5 / 1.48e-4`，但 Fn-labelled
increments material 地进入该缺失方向：

- Fn_L：left/right unrealizable fraction `0.35798 / 0.33664`；
- Fn_R：left/right unrealizable fraction `0.79030 / 0.04910`。

不可实现分量对 QP predicted rolling cancellation 的 contribution fraction 约为
`0.9995–1.0249`。去掉该正交分量后的 minimum-norm point forces 在 nominal load 上仍保持
normal nonnegative 与正 friction margin，所以 gate 不是由单独 `ΔFn<0`、friction switch 或
unilateral loss 触发，而是 aggregate wrench column-space 本身不匹配。

MuJoCo actual response 保持同一 contact/solver regime，并正常抵消 `98.79–98.93%` 的
torque-induced rolling free tendency。裸 Delassus block 仅作为 effective-mass diagnostics，
没有被误写成完整 compliant solver law。

证据：[point realizability](evidence/automated/root-cause-closure-formal-v3/point-realizability.json)、
[solver response](evidence/automated/root-cause-closure-formal-v3/solver-response-operator.json)。
