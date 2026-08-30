# Phase 44 Addendum — Contact Directional Transfer Audit

For every trusted direction the oracle reports `Delta Q_act`, controller `Delta Q_contact_QP`,
MuJoCo `Delta Q_contact_MJ`, `M Delta qacc`, and the remaining generalized-force term in the same
reduced/native wheel-row construction. Across all 612 probes:

- whole-vector dynamics residual maximum: `1.77636e-13`;
- contact reconstruction residual maximum: `0`;
- directional wheel-row balance residual maximum: `0`.

In the bounded trusted `D/native_common` scope, all 27 available signed directions have opposite
actuator/contact signs. `-Delta Q_contact_MJ / Delta Q_act` lies in `0.76107..0.90170`, so actual
contact reaction consistently cancels or redirects most actuator wheel-row authority. At tick0 the
ratio is about `0.89883..0.90030` on both sides and signs.

This upgrades the mechanism statement to:

> controller-predicted contact generalized-force realization and actual MuJoCo
> contact-constrained realization differ materially; contact authority cancellation/redirection is
> a primary controller-to-plant mechanism in trusted D/native-common probes.

It does not claim that every task channel has the same cancellation ratio, nor that the MuJoCo
contact solver or controller contact allocation code is wrong.

