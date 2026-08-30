# Contact Realization Audit

Evidence: [`contact-generalized-force.csv`](evidence/automated/realization-audit-formal-v1/contact-generalized-force.csv),
[`contact-authority-transfer.csv`](evidence/automated/realization-audit-formal-v1/contact-authority-transfer.csv)

The primary comparison is strictly reduced generalized force:

- QP: `Q_contact_QP = W_L*lambda_L + W_R*lambda_R`.
- MuJoCo: `Q_contact_MJ = N^T*(qfrc_contact_left + qfrc_contact_right)` reconstructed per contact with
  `mj_contactForce` and `mj_applyFT`.

No arbitrary 12D-to-16D force lift and no raw-lambda equality claim is used. Contact reconstruction closes within
the inherited `1e-8` tolerance.

At shared tick0 the generalized-force relative difference is `0.07731`; it grows to `0.33322` over audited
snapshots. The wheel-row absolute differences are smaller than the base/leg aggregate but the local transfer is
material: for D/native-common tick0, per unit task input, actuator wheel authority is about `+2.408e-4`, while
MuJoCo contact authority is about `-2.166e-4`; most direct actuator generalized authority is cancelled before
native wheel acceleration, which has gain only about `-0.064`. This is strong provisional `B-contact` evidence.

The table records `M_w Delta qacc`, `Delta Q_act`, `Delta Q_contact` and the remaining constraint/passive term for
each side/channel/snapshot. It also shows that B/native-common achieves its QP wheel row mainly by changing the
optimized contact realization: actual actuator gain is nearly zero and native wheel gain reverses to about
`-0.509` at tick0.
