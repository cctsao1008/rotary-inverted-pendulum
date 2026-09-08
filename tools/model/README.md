# Model Tools

`tools/model` owns host-side model analysis that is deliberately independent of the production Rust dynamics implementation.

The nonlinear Furuta correlation path uses an explicit reference-backed nominal fixture. `reference_furuta.py` rebuilds the nonlinear equations in Python/NumPy and integrates them with SciPy DOP853 in float64, while `tools/sitl` exposes the production `plant/dynamics-model` f32 RK4 trace for the same initial state and zero-order-held arm-torque profile.

Correlation demonstrates implementation consistency between independent numerical realizations. It does not establish Forest D1 specimen calibration or replace the parameter source/applicability semantics in `parameters/reference-assembly.json`.
