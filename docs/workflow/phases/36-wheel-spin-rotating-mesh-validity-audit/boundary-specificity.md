# Boundary specificity

The frozen local test compares ddxi slopes over `0.95→0.99`, `0.99→1.01`, and `1.01→1.05 rad`
for both signs and all three phase modes. A natural boundary requires a central/neighbor slope ratio
of at least `5` and an absolute central jump of at least `0.05 m/s²`.

Observed ratios are:

| mode | negative | positive |
| --- | ---: | ---: |
| left | 0.492 | 2.627 |
| right | 1.040 | 3.972 |
| bilateral | 0.929 | 2.648 |

None reaches `5`. Contact counts can change within this neighborhood (as they do elsewhere in the
sweep), but neither sign shows a repeatable discontinuity uniquely at exactly one radian. All core
geometry/model quantities remain continuous and periodic.

DG36-04: `±1 rad` boundary-specificity **not evidenced**.
