# Model Tools

`tools/model` owns host-side model analysis that is deliberately independent of the production Rust dynamics implementation.

The nonlinear Furuta correlation path uses an explicit reference-backed nominal fixture. `reference_furuta.py` rebuilds the nonlinear equations in Python/NumPy and integrates them with SciPy DOP853 in float64, while `tools/sitl` exposes the production `plant/dynamics-model` f32 RK4 trace for the same initial state and zero-order-held arm-torque profile.

`rigid_body/` defines a second, structurally independent validation path. Its machine-readable contract fixes the simulator/world axes, project state order `[theta, theta_dot, phi, phi_dot]`, joint signs, arm-torque sign, and falsifiable near-upright direction predictions. `build_furuta_urdf.py` renders a Furuta URDF directly from `parameters/reference-assembly.json`; the URDF therefore consumes the project nominal parameter boundary without becoming another parameter authority. Simulation-only numerical completion needed to keep the articulated rigid body well-formed is declared separately in `rigid_body/furuta_contract.json`.

The external rigid-body path must not call or reproduce the project nonlinear derivative function. Its purpose is to pressure-test equation structure, coordinates, signs, and geometry using an independent rigid-body solver. A positive arm torque at upright is predicted to produce positive `phi_ddot` and negative `theta_ddot` under the declared coordinate contract; small unforced positive/negative `theta` near upright must accelerate further in the same sign because upright is unstable.

Generate the nominal URDF without a physics-engine dependency:

```bash
python tools/model/rigid_body/build_furuta_urdf.py \
  --output target/model-rigid-body/furuta.urdf \
  --manifest target/model-rigid-body/manifest.json
```

The PyBullet validation dependency is isolated in `requirements-rigid-body.txt`. With it installed, produce a headless trace from the same nominal correlation fixture:

```bash
python tools/model/rigid_body/pybullet_furuta.py \
  --fixture tools/model/fixtures/reference_nominal_correlation.json \
  --output target/model-rigid-body/pybullet-trace.json
```

The runner requires the fixture plant values to match `parameters/reference-assembly.json`, disables Bullet's default joint motors, applies only the declared arm-torque profile, and exports the canonical project state order. PyBullet is a validation dependency only and is not part of production control or firmware.

Correlation demonstrates implementation or model-structure consistency only. It does not establish Forest D1 specimen calibration, replace the parameter source/applicability semantics in `parameters/reference-assembly.json`, or grant physical motor authority.
