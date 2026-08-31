# QP-vs-MuJoCo Smooth / Pre-Contact Dynamics Attribution

## Decision

`C1-RAW-MASS-INERTIA-RESPONSE-MISMATCH`。本轮仅作 fixed-state attribution；controller、QP、
corrected Pg、torque和physical contact wrench均未修改。

## First-mismatch chain

slip-common total gap经 contact、legal common equality、MJ-only equality和numerical remaining扣除后，
target remainder为 `-0.388661935034175`。raw-tree mass diagnostic独立重现
`-0.388661935034160`，bookkeeping residual `<=5.56e-13`。

上游 gates全部通过：MuJoCo input严格等于 `-tau_QP`，maximum torque application error `0`；
`B_QP*Delta tau` 与 MuJoCo `Delta qfrc_actuator`逐值相同，gap `0`；other smooth-force gap
`<=1.78e-13`；q/qdot/M directional deltas为numerical zero。因此 target可以升级为真实的
raw-tree FREE RESPONSE GAP。

同一 generalized force下，production与MuJoCo full-tree mass matrices的max difference
`0.497927324`、relative Frobenius difference `0.096988856`、spectral difference `0.585684816`；
raw qacc gap norm `1.016570084`，其 observable slip-common projection解释目标的
`99.999983%`。最大qacc gap位于base-z translation。per-actuator slip-common contributions中
RH为 `-0.653811267`、LH为 `+0.292795751`；family signed shares为hip `92.884%`、knee
`-0.907%`、wheel `8.023%`。

slip-differential raw precontact self gap仅 `-0.000129902`，故该 failure是common-mode
specific。xi-common自身输出gap仅 `-0.000880417`；其mass discrepancy主要投影到slip-common
cross channel (`+0.065686918`)，随后由contact response抵消，所以最终xi AUTH仍健康。

按first-mismatch stop rule，closure-conditioned rank4/rank6 oracles与observable parity均
`NOT_REACHED`，不会以downstream结果重写C1。same-torque branch split `2.64e-10`、scale
convergence `1.23e-9`，fresh replay error `0`。contact仍material，legal equality不material，
contact不是唯一剩余机制；R2不授权。

Evidence：
[formal-v3](evidence/automated/precontact-free-response-attribution-formal-v3/precontact-free-response-attribution.json)、
[fresh replay-v1](evidence/automated/precontact-free-response-attribution-replay-v1/summary.json)。
