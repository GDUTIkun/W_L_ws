# Phase44 Decision

Formal classification: `P44-U`  
Provisional layered finding if DG44-06 is repaired: `P44-E` with contact mechanism evidence.

## Five required answers

1. **`a_des -> a_QP`:** tick0 loss is negligible (`~1e-10` to `1e-9` normalized), so it does not explain the
   initial failure. Later snapshots develop material QP loss, up to `18.7863` normalized; optimization is a later
   contributing layer.
2. **`nudot_QP -> qacc_MJ`:** material loss exists immediately. Tick0 right-wheel error is about
   `-3.087 rad/s^2`; later native/xi errors become much larger. The affine closed-chain bias is included.
3. **Contact generalized force:** reduced QP/MuJoCo contact differs by `7.73%` at tick0 and up to `33.32%` in
   scope. D/native probes show actuator authority largely cancelled/redirection through contact reaction.
4. **Why C improves xi:** the actual xi acceleration is dominated by leg/wheel-center motion; native wheel-spin
   contribution is zero in this coordinate. Tick0 plant xi self gain is still about `0.948` while native response
   reverses relative to QP prediction.
5. **Minimum future rolling variables:** a contact-consistent combination of xi, native wheel spin and tangential
   slip/contact load. No Phase44 repair is authorized.

## Why the decision remains unresolved

Task, affine acceleration, contact reconstruction, rolling definitions and xi decomposition are valid, and formal
equals fresh replay. But the required authority matrices violate frozen symmetry/half-delta gates at late
snapshots. Because authority-matrix trust is explicitly mandatory for P44-A/B/C/E, the correct formal result is
P44-U rather than upgrading the strong multi-layer evidence to P44-E.

The next work must be a Phase44 addendum/rework that freezes a regime-aware derivative oracle (for example one-sided
directional derivatives tied to unchanged active/contact sets). It must not tune gains/weights or implement repair.
