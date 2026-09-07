# Physical Parameter Registry

`reference-assembly.json` is the machine-readable nominal parameter set for the Rotary Inverted Pendulum reference assembly.

Each parameter carries:

```text
value
unit
source
applicability
```

The registry separates production-model assumptions from virtual physical truth. `measurement_model` and `production_actuator_model` define the assumptions consumed by production semantics; `virtual_sensor` and `virtual_physical_actuator` define the simulated physical world used by SITL. They may intentionally differ when testing model uncertainty or robustness.

The parameter set is reference-backed nominal system definition. It is not a claim that every value is a specimen-specific measurement of the installed hardware.
