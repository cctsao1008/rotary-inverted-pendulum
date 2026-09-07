# Tools

Host-side tools live under this directory and do not define an additional architecture domain.

```text
sitl/
```

`tools/sitl` owns deterministic virtual time, scenario execution, virtual physical truth, machine-readable evidence, and CI-facing Software-In-The-Loop execution. Production estimator, controller, actuator-model, runtime-authority, and Firmware actuator semantics are reused from their owning domain crates rather than duplicated here.
