# Tools

Host-side tools live under this directory and do not define an additional architecture domain.

```text
model/
sitl/
```

`tools/model` owns independent host-side model analysis and correlation. Its Python/NumPy-SciPy Furuta reference does not import or execute the production Rust dynamics implementation; both sides consume an explicit correlation fixture so equation, sign, and integration drift can be detected without claiming specimen calibration.

`tools/sitl` owns deterministic virtual time, scenario execution, virtual physical truth, machine-readable evidence, and CI-facing Software-In-The-Loop execution. Production estimator, controller, actuator-model, runtime-authority, and Firmware actuator semantics are reused from their owning domain crates rather than duplicated here.
