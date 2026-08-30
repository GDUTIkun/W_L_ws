# Rigid-body phase isolation

Contact-off within-cycle modulation and ratios to V0:

| Quantity | V0 | V1 centered COM | V1/V0 | V2 symmetric inertia | V2/V0 |
| --- | ---: | ---: | ---: | ---: | ---: |
| mass matrix | `7.9718e-5` | `3.7523e-8` | `4.707e-4` | `7.9718e-5` | `1.000` |
| bias | `7.8203e-4` | `0` | `0` | `7.8203e-4` | `1.000` |
| generalized acceleration | `4.17019` | `5.4028e-7` | `1.296e-7` | `4.17019` | `1.000` |
| physical ddxi (m/s²) | `1.3384e-4` | `9.5383e-8` | `7.127e-4` | `1.3380e-4` | `0.9997` |

V3 reduces mass modulation to `4.07e-20`, bias and contact-off response to numerical zero, but V1
already passes both rigid-body and response causal gates. V2 alone changes nothing material.

DG38-04: **PASS — COM eccentricity is the dominant numerical source; transverse anisotropy is not**.
