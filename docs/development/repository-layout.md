# Repository Layout

The active source tree is Rust-first.

```text
Cargo.toml
rust-toolchain.toml
.cargo/

crates/
├── control/             Pure `no_std` control computation
├── supervisor/          Runtime supervision, ports, and motor authority
└── plant/               Plant conversions and drive conventions

firmware/
└── stm32f103/           STM32F103 composition root and linker memory definition

docs/
├── architecture/        Architecture and interface definitions
├── hardware/            Hardware definition and provenance
└── development/         Repository/build reference
```

The previous C application, C control core, board API, libopencm3 platform implementation, CMake build, legacy drivers, and C host tests are not part of the active tree. Git history retains them.

## Dependency direction

```text
control
  ▲
  │
plant ◄── supervisor
  ▲          ▲
  └────┬─────┘
       │
    firmware
```

`control` is independent of target hardware and physical actuator representation. `plant` may depend on control-domain value types. `supervisor` composes control and plant semantics. Firmware implements target-specific ports and owns hardware integration.
