# H0 production-contract revalidation

DG41-02: **PASS / P41-A**.

Authority: Model B, Phase27 Minimal, frozen fixed equilibrium interaction wrench, no xi task, no
target motion, zero gains. `phase41_workspace_contract_loop` calls the normal production
`NominalWbcModel::evaluate(state)` and `WeightedWbcController::step(state, reference)` APIs; there is
no policy argument or diagnostic bypass.

| Event | Production result |
| --- | --- |
| historical wheel ±1 location | tick 96 |
| model status at tick 96 | OK |
| continued after tick 96 | yes |
| first independent failure | tick 111, right contact loss only |
| valid frozen gates before failure | all PASS |
| maximum wheel rotation at stop | 2.848143533837517 rad / 0.4532961220 rev |
| formal/replay max error | 0 |
| Phase40 shadow physical/control parity error | 0 |

The run stops at tick111 exactly as frozen; it does not continue to later reconstruction/slack/base
failures. This promotes the Phase40 diagnostic finding to production-contract evidence: the false
workspace blocker is removed and the next real blocker in H0 is wheel-spin drift trajectory leading
to contact loss. Causality and repair are not decided here.
