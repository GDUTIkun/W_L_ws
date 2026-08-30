# Inertial counterfactual contract

All branches load a fresh Phase37 cylinder model and change only compiled diagnostic copies:

- V0: current compiled mass, COM, tensor and inertial frame;
- V1: set body-frame COM X/Y to zero; preserve COM Z, mass, principal inertia and `body_iquat`;
- V2: preserve COM/mass; set body-frame tensor to
  `diag((Ixx+Iyy)/2,(Ixx+Iyy)/2,Izz)`;
- V3: apply V1 and V2.

Each branch calls `mj_setConst`, and the manifest records exact before/after arrays. No XML or
production parameter is changed. V4 is absent because L/R mismatch did not exceed the pre-frozen 5%
gate. A primary factor must reduce contact-off physical-ddxi to `≤0.2×` baseline and also reduce at
least one of mass/bias modulation to `≤0.2×`.

These are causal counterfactuals, not candidate real-wheel parameters.
