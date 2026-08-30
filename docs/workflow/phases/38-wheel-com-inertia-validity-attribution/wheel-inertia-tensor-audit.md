# Axle-aligned inertia tensor audit

Compiled COM-centered tensors in the wheel body/axle frame are approximately:

```text
left  [[2.4588192e-4, 5.4621e-9, 0],
       [5.4621e-9, 2.4589840e-4, 0],
       [0,            0,       4.2373759e-4]]

right [[2.4588027e-4, 0, 0],
       [0, 2.4590005e-4, 0],
       [0, 0, 4.2373759e-4]] kg·m²
```

Transverse anisotropy is `6.70e-5` left and `8.04e-5` right, both below the frozen `1e-4`
significance threshold. The only visible product, left `Ixy=5.46e-9 kg·m²`, is normalized
`1.79e-5`; axle products are numerical zero. Full-tensor L/R mismatch is `1.289e-5`, far below 5%.

The axle is therefore a principal axis to numerical precision and the tensor is already nearly
axisymmetric. DG38-02: **PASS; no inertia-frame or L/R inconsistency finding**.
