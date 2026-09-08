# Physical Parameter Registry

`reference-assembly.json` is the machine-readable nominal parameter set for the Rotary Inverted Pendulum reference assembly.

Each parameter carries:

```text
value
unit
source
applicability
```

`reference-assembly-evidence.json` is the provenance sidecar for that registry. It records the evidence class, source identity, specimen applicability, derivation relationship, and rationale for every numeric parameter without becoming a second source of parameter values.

The registry separates production-model assumptions from virtual physical truth. `measurement_model` and `production_actuator_model` define the assumptions consumed by production semantics; `virtual_sensor` and `virtual_physical_actuator` define the simulated physical world used by SITL. They may intentionally differ when testing model uncertainty or robustness.

The provenance sidecar distinguishes reference-backed nominal values, values derived from references, vendor-documented hardware-family facts, standard constants, project references/conventions, explicit model assumptions, and simulation fixtures. Numerical agreement or vendor-family documentation does not by itself promote a value to specimen-specific calibration.

The parameter set is a reference-backed nominal system definition. It is not a claim that every value is a specimen-specific measurement of the installed hardware.
