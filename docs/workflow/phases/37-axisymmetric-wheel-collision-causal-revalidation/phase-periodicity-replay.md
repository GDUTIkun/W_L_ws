# Phase periodicity replay

The Phase36 corpus was replayed against the collision-only revision:

- core `q/q+2π` maximum error: `3.47e-18`;
- contact/dynamic `q/q+2π` maximum error: `2.30e-13`;
- periodic contact topology: PASS;
- no material contact geometry modulation;
- no special `±1 rad` transition.

The reused Phase36 classifier therefore returns
`P36-C_periodic_consistent_bound_unsupported`. This confirms robust periodicity and independently
continues to reject one radian as a natural model boundary.

DG37-02: **PASS**.
