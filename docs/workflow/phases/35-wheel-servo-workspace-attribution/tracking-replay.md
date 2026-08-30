# Phase 35 Phase34 tracking replay

The exact three frozen gains crossed with step/ramp all reproduce `kOutsideWorkspace`. Rejecting
ticks are `[91,92,91,90,92,92]`, inside Phase34's authoritative `[90,92]` interval. Every case first
fails canonical index 5 at its lower wheel-angle bound and retains the invalid sample.

This reproduces the Phase34 mechanism while showing it is not introduced by commanded tracking:
H0 fails earlier at tick 88. DG35-05: **PASS**.
