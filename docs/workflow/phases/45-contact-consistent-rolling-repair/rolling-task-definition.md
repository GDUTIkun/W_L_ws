# Unified rolling task definition

每侧 residual 为 `e_roll=[dxi + Kxi(xi-xi0), v_t]`。WBC acceleration targets分别为
`ddxi*=-Kxi_p(xi-xi0)-Kxi_d dxi` 与 `a_t*=-Kslip v_t`。material point、normal、tangent、
`v_C+omega cross r`、affine row均来自同一 current MuJoCo contact observation；ground velocity为0。
Phase45 profile把两类 row一次加入同一42-variable QP，不创建 xi-only、slip-only 或 native-rate candidate。
