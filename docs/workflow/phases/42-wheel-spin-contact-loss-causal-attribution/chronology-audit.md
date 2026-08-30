# Phase 42 Chronology Audit

Authority: `causal-attribution-formal-v1/events.json`; fresh replay is identical.

## Result

The 2 ms chronology reproduces bilateral contact through tick110 and the first right-wheel contact
loss at tick111. At tick0, before rate drift exists, right-minus-left normal load is already
`0.5366958458 N` and right-minus-left penetration is `-0.1264461 mm`. The fixed command also gives
physical common/differential wheel acceleration of `-0.1202064/-0.0214450 m/s²`.

At the nominal material floors, common wheel rate and leg displacement first persist at tick1,
base-position change at tick4, differential rate at tick7, differential acceleration at tick9,
common acceleration at tick47 and base speed at tick74. These are detector results, not an
unqualified causal ordering.

## Sensitivity limit

Most onset ticks change across `0.5x/1x/2x` floors; for example common-rate onset is `1/1/16`,
base-position onset is `3/4/6`, and common-acceleration onset is `6/47/75`. Therefore no strict
total ordering among wheel, leg and base drift is approved. The sensitivity-stable facts are the
tick0 contact/penetration asymmetry, tick0 instantaneous non-equilibrium, subsequent bilateral
evolution through tick110, and tick111 terminal loss.

