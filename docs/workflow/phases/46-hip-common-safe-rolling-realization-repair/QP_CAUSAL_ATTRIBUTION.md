# QP causal attribution

Verdict: `T2-ACCELERATION_TASK_COUPLING_DOMINANT`.

在项目真实约定下，QP increment identity 为
`M Δnudot = B Δtau + Jw^T Δlambda`，formal-v2 最大闭合误差 `3.392e-9`。
由同一方程的唯一 `B+` operator 定义，Fn_L/Fn_R 的 acceleration-component torque norm
占完整 `Δtau` 的 `94.9461% / 93.7565%`；contact-balancing norm share 为
`7.0881% / 9.8319%`，other
只剩 `2.63e-10` 数值量级。

fixed-active-set KKT sensitivity 进一步确认：真实 RHS excitation 是 wheel-longitudinal 与
rolling/slip target；`Fn_L/Fn_R` 是通过 command-space calibration 得到的 QP solution-space
方向标签，不是直接施加给 plant 的 aggregate Fn input。xi 与 rolling 两组 excitation 经完整
KKT operator 传递后产生强 bilateral hip/knee torque。该事实解释 torque 为什么大，但本身不等于
task formulation 已被证明错误。

证据：[formal-v3](evidence/automated/root-cause-closure-formal-v3/qp-torque-source.json)、
[KKT](evidence/automated/root-cause-closure-formal-v3/kkt-sensitivity.json)。
