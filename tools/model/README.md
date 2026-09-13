# Model Tools

`tools/model` owns host-side model analysis that is deliberately independent of the production Rust dynamics implementation.

The nonlinear Furuta correlation path uses an explicit reference-backed nominal fixture. `reference_furuta.py` rebuilds the nonlinear equations in Python/NumPy and integrates them with SciPy DOP853 in float64, while `tools/sitl` exposes the production `plant/dynamics-model` f32 RK4 trace for the same initial state and zero-order-held arm-torque profile.

## Model authority

The project uses model authority by **task**, not by declaring one simulator or equation set globally correct.

- The production Rust `FurutaPlant` and `reference_furuta.py` represent the same source-backed reduced/equivalent QNET nonlinear model. Their agreement is reduced-equation implementation consistency. This model remains appropriate for upright linearization, local nominal controller synthesis, and fast deterministic semantic-path SITL within an explicitly characterized near-upright envelope.
- `rigid_body/full3d_furuta.py` is a geometry-derived articulated 3-D analytical model. It exists as a separate validation model class for configuration-dependent inertia and velocity-coupling effects that the reduced model omits. Capture and swing-up dynamic conclusions require this richer model class or an independent rigid-body backend rather than reduced-model prediction alone.
- PyBullet is an independent rigid-body implementation used to pressure-test the declared geometry, coordinates, and full-3D analytical equations. Agreement between full-3D analytical dynamics and PyBullet is model-structure/backend evidence, not installed-mechanism truth.

`fixtures/reduced_model_validity_envelope.json` and `check_reduced_model_validity_envelope.py` own the quantitative engineering boundary between local reduced-model use and regions that require full-3D cross-checking. The decision margin is explicitly a model-selection criterion; it is not a measured Forest D1 tolerance. A future parameter/model change must update the executable evidence rather than silently broadening model authority.

The reduced model may therefore remain the default for deterministic software/semantic validation without being granted large-angle predictive authority. The full-3D model may be more complete geometrically without becoming specimen calibration. Neither model grants physical actuator authority.

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

Run the prediction-first coordinate/input checks separately:

```bash
python tools/model/rigid_body/check_causality.py
```

These checks deliberately fail on a persistent sign disagreement: a small positive/negative pendulum displacement must diverge in the corresponding direction near upright, and positive arm torque at upright must produce positive arm rate with negative pendulum rate under the declared coordinates. Such a failure is treated as a coordinate/geometry/input-contract defect, not as a controller-tuning problem.

The runner requires the fixture plant values to match `parameters/reference-assembly.json`, disables Bullet's default joint motors, applies only the declared arm-torque profile, and exports the canonical project state order. PyBullet is a validation dependency only and is not part of production control or firmware.

Correlation demonstrates implementation or model-structure consistency only. It does not establish Forest D1 specimen calibration, replace the parameter source/applicability semantics in `parameters/reference-assembly.json`, or grant physical motor authority.
