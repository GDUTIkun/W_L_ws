# Architecture evidence update

## Evidence still valid

- Phase 31 measurement/kinematics contracts；
- Phase 32 pair construction、projection、full-body oracle 与 Model B 上通过的 validity gates；
- Phase 36 rotating collision-mesh artifact 和 Phase 38 radial-COM numerical causality；
- Phase 35 在原 Model A/mesh plant 上的历史观测，作为 mismatch evidence 保留。

## Superseded for causal interpretation

rotating mesh 和 eccentric COM 引起的 absolute wheel-phase difference 不再能单独作为 x16
intrinsic non-closure 或 12D architecture 的证据。Model B 上 wheel-angle family 已通过。

## Revalidated evidence

Model B 上 C1/C2/C3 分别以 `0.08480/2.07846/1.65810 m/s²` 继续失败；absolute angle 以
`6.045e-5 m/s²` 通过。requested 与 realized wrench parity 均通过，angle fixed-torque branch
也通过。H0 wheel drift 和 right-wheel workspace chronology 仍存在。

因此 `P39-D` 加强 `12D base NMPC + full-body WBC wheel realization` 的 responsibility-split
candidate，但不批准其 tracking/robustness，也不把旧 Eq.(12) 或任何新 architecture 设为
production。公平 tracking 比较仍被未解决的 workspace state/domain contract 挡住。
