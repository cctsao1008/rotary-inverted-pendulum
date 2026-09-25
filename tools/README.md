# Tools

Host-side tools live under this directory and do not define an additional architecture domain.

```text
model/
sitl/
telemetry/
visualization/
rp2350_commission/
```

`tools/model` owns independent host-side model analysis and correlation. Its Python/NumPy-SciPy Furuta reference does not import or execute the production Rust dynamics implementation; both sides consume an explicit correlation fixture so equation, sign, and integration drift can be detected without claiming specimen calibration.

`tools/sitl` owns deterministic virtual time, scenario execution, virtual physical truth, machine-readable evidence, and CI-facing Software-In-The-Loop execution. Production estimator, controller, actuator-model, runtime-authority, and Firmware actuator semantics are reused from their owning domain crates rather than duplicated here.

`tools/telemetry` owns host-side decoding/inspection of recorded production evidence.

`tools/visualization` owns read-only presentation of machine-readable simulation, model, telemetry, and evidence outputs. Visualization may explain evidence but does not produce dynamics, control decisions, model authority, specimen calibration, or physical actuation authority.

`tools/rp2350_commission` owns the host-side RP2350A commissioning session. One CLI coordinates both USB HID machine telemetry and CDC diagnostic logging, records evidence under `artifacts/commissioning`, and groups passive sensor checks, bounded motor characterization, position tests, and SysID excitation behind one operator-facing entry point.
