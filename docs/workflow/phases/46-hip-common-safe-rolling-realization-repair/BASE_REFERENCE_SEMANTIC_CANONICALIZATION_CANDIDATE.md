# Base Reference Semantic Canonicalization Candidate

## Decision

`A-EXACT-BASE-REFERENCE-CANONICALIZATION-CANDIDATE`，仅授权下一轮实现一个 cross-model
diagnostic-boundary canonicalization；本轮没有修改 production。

MuJoCo `FRAME M` 是 `base_body/free-joint` origin，production `FRAME P` 是
`base_control_frame`。authoritative H0下二者orientation均为world identity；真实site geometry给出
`r_M_to_P=[-0.077378152, 8.1e-7, -0.03227768] m`。configuration law为
`p_P=p_M+R_M^W r_M_to_P^M`、`R_P=R_M`、full-tree joints identity。body pose/rotation parity为
`3.93e-17 m / 2.76e-16 rad`，configuration FD error最大 `4.34e-9`。

由configuration differential得到 `nu_P=X_PM nu_M`。`X`双向closure为zero；body/site twist
covariance `<=2.35e-16`，virtual-power residual `4.44e-16`。一般nonzero rotation的acceleration FD
error为 `4.48e-10`；H0 velocity为zero，故本state `Xdot*nu=0`，不是未经验证地假定`Xdot=0`。

## Same-model covariance

同一个production physical model在P/M coordinates下：mass relative/spectral/max gaps分别
`3.08e-17/1.92e-16/1.11e-16`，kinetic-energy error `3.55e-15`，bias/full-EOM errors均
`1.78e-15`，Jacobian error `2.35e-16`，reduction error `2.83e-16`。因此production内部
control-point semantics本身自洽。

FIRST WRONG SEMANTIC CONSUMER不是`NominalWbcModel`，而是Phase46 cross-model response attribution
把production control-point `M/qacc` 与MuJoCo body-origin `M/qacc`作为同一16D coordinates并复用
未变换observable map。最小插入点是cross-model diagnostic comparison boundary；只 canonicalize
一次 `q/qvel/qacc/M/h/Q/J/N/c_N/observable`，production controller和external contract不变。

## Candidate closure

common4 gap从 `-0.3883828695` 降至 `-0.00861745576`，移除
`-0.37976541375`，signed/absolute fraction均为 `97.7811957%`。remaining与冻结inertial-family
effect `-0.00861756963` 相差 `1.14e-7`，一致。

candidate后matched-tangent mass relative gap为 `1.53638e-5`，effective-operator relative gap
`5.74414e-4`，dominant tangent kinetic-energy gap `-1.01665e-6`。MJ-only rank-2 closure effect在
candidate之外；contact formulation未触碰、bookkeeping未回归、physical response未重算。

Evidence: [formal-v2](evidence/automated/base-reference-candidate-formal-v2/base-reference-semantic-canonicalization-candidate.json)
and [fresh replay-v1](evidence/automated/base-reference-candidate-replay-v1/summary.json).
