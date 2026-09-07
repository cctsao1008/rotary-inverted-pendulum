# Repository Layout

The repository exposes the same top-level control-system grammar used by `single-wheel-platform`. Domain ownership is visible in the path; plant-specific and controller-specific leaves remain project-specific.

```text
Cargo.toml
rust-toolchain.toml
.cargo/

plant/
├── robot-domain/
├── dynamics-model/
├── measurement-model/
├── plant-observation/
└── actuator-model/

control/
├── state-feedback/
└── hybrid-control/

supervisor/
├── state-estimator/
├── runtime-state/
└── control-runtime/

firmware/
├── interfaces/actuation/
├── actuators/tb6612/
├── adapters/estimator-input/
└── targets/stm32f103/

support/
└── dsp-kernel/

parameters/
├── README.md
└── reference-assembly.json

tools/
└── sitl/

docs/
├── architecture/
├── hardware/
└── development/
```

The production architecture remains exactly:

```text
plant/
control/
supervisor/
firmware/
```

`support/`, `parameters/`, `docs/`, and `tools/` are repository support areas rather than additional production architecture domains. Only leaves with implemented system content are materialized.

The Firmware taxonomy remains `interfaces / sensors / communications / ui / buses / actuators / adapters / boards / assemblies / targets`; this project currently materializes only the leaves required by its implemented hardware path.

`support/dsp-kernel` owns cross-domain numerical implementation primitives. Production ARM builds use the target DSP backend while host builds preserve deterministic semantic behavior for tests and SITL. It owns no Plant, Control, Supervisor, or Firmware semantics.

## Dependency direction

Portable dependency flow follows semantic ownership:

```text
Plant semantics
   ▲      ▲
   │      │
Control  Supervisor
   ▲        ▲
   └────┬───┘
        │
     Firmware
```

Control consumes Plant semantics. Supervisor composes Plant and Control behavior while owning estimation and authority. Firmware depends on the portable domains and owns physical realization. Production-domain crates may depend on narrowly scoped `support/` implementation primitives, but `support/` must not depend back on production-domain semantics.

`firmware/adapters/estimator-input` converts Plant-owned raw ADC/encoder evidence into the Supervisor estimator input representation. `firmware/targets/stm32f103` materializes the sensing, estimation, control, actuator-model, and authority computation path without linking a physical actuator sink.

`tools/sitl` is host-side verification infrastructure rather than a fifth architecture domain. It provides deterministic virtual time, scenario execution, machine-readable evidence, the virtual Furuta plant/sensor/actuator world, and reuse of the production semantic path through the Firmware TB6612 actuator adapter.
