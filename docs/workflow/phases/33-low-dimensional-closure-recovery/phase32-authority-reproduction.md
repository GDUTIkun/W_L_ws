# Phase 32 Authority Reproduction

DG33-00: **PASS**.

Using `./.venv/bin/python` (`Python 3.10.20`, MuJoCo `3.7.0`, NumPy `2.2.6`,
SciPy `1.15.3`), Phase32's four independent runners were executed into new Phase33 output
directories. The new summaries are byte-identical to the final Phase32 replays:

| Authority | Phase32/Phase33 summary SHA-256 |
| --- | --- |
| floating-base Markov/C3 | `1dd8f07b7e3842a96fde5490a3003719e55906ed8cc2df50c012ff9310fef68d` |
| C1/C2 leg nullspace | `5a82a26ec1dc8f3670ab436846e9f91699d3326129e2c98c6caa4243d552004a` |
| projection rank | `8acb14d40119bb8bcc9c5fcdc11222c5b62ffd8d34d1527ab05650fa2e041e54` |
| wheel-angle hybrid | `d3667407db5051e3575033a1b744cff728e18fe0bcdbfe14508457b14615daec` |

The unmodified `test_phase27_minimal_wbc` also passed with 42 variables, 104 hard rows,
`max=0.560444 ms` in the pre-change run. Its post-change regression remains PASS.

This reproduces the Phase32 finding; it does not approve x24 or reinterpret algebraic WBC wrench
realization as a directly measured physical interaction wrench.
