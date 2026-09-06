# Repository Layout

The active source tree is Rust-first and exposes the four architectural domains directly at repository root.

```text
Cargo.toml
rust-toolchain.toml
.cargo/

plant/
├── robot-domain/
├── plant-observation/
├── measurement-model/
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
└── targets/stm32f103/

docs/
├── architecture/
├── hardware/
└── development/
```

There is no generic top-level `crates/` container. Domain ownership is visible in the path itself.

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

Control consumes Plant semantics. Supervisor composes Plant and Control behavior while owning estimation and authority. Firmware depends on the portable domains and owns physical realization.

The previous C implementation and superseded Rust layout are retained in Git history rather than in the active source tree.
