# Superseded Phase 22 formal v1

The simulation checks passed (19/19 normal and 6/6 fault), but this run is not
final authority. The config overlay inherited obsolete Phase 21 ADMM metadata
(`rho` and `over_relaxation_alpha`) into the manifest's otherwise-correct
ProxQP solver block. No product behavior or gate was affected, but solver
identity must be unambiguous. The output is preserved under the non-overwrite
rule and superseded by `2026-08-28-formal-v2`.
