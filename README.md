# 🌀 Rotary Inverted Pendulum

> **A ground-up re-architecture of a rotary inverted pendulum control system, from physical I/O to hybrid control.**

The repository defines the control-system architecture for a rotary inverted pendulum. Hardware access, sensing, actuation, state estimation, control, mode ownership, telemetry, safety, and physical actuator authority are explicit boundaries.

The physical plant is the starting point. Controller algorithms are replaceable implementations inside the architecture.

## Architecture

```text
Physical Plant
    ↓
Platform / Hardware Contract
    ↓
Sensing & Actuation
    ↓
State Estimation
    ↓
State Safety
    ↓
Control State Machine
    ↓
Controller Dispatch
    ↓
Actuator Mapping / Output Safety
    ↓
Computed Actuator Command
```

Physical motor ownership is separate from control computation:

```text
Maintenance / Control Request
        ↓
Motor Authority Arbiter
        ↓
board_motor
        ↓
TB6612
        ↓
Rotary-Arm Motor
```

**Computing a control command is not equivalent to having authority to move the motor.**

## Implemented system

| Component | Implementation |
|---|---|
| Embedded platform | STM32F103 |
| Shared hardware contract | `platform/api/` |
| State estimator | Basic estimator |
| Balance controller | LQR |
| Automatic-control profile | Observe-only |
| Automatic motor sink | Unbound |
| Physical motor path | Bounded maintenance path through `motor_authority` |
| Maintenance transport | Text UART |

## Platform boundary

```text
platform/
├── api/            Shared board contract
├── stm32f103/      STM32F103 implementation
└── rp2350/         Reserved platform namespace
```

The legacy Forest D1 / Forest S1 names are used only for original hardware and documentation provenance. They do not define the software architecture.

## Reference physical plant

- STM32F103C8T6 reference controller
- geared nominal 12 V rotary-arm DC motor
- quadrature Hall encoder
- conductive-plastic pendulum angular-position sensor
- TB6612FNG H-bridge
- original rotary inverted-pendulum mechanical plant

See [Forest D1 2016 Hardware Baseline](docs/hardware/forest-d1-2016-baseline.md).

## Repository ownership

```text
app/                    Application integration and system orchestration
control/                Platform-independent estimation, control, safety, and mode logic
drivers/                Reusable device drivers
platform/api/           Shared hardware contract
platform/stm32f103/     STM32F103 implementation
platform/rp2350/        Reserved platform namespace
tests/                  Host-side deterministic tests
tools/                  Runtime and analysis tooling
docs/architecture/      Architecture and interface contracts
docs/commissioning/     Firmware and maintenance interfaces
docs/control/           Controller implementation
docs/hardware/          Hardware definition
docs/development/       Repository and build reference
```

## Documentation policy

Markdown describes only the resulting system:

- architecture;
- interfaces and contracts;
- implemented behavior and reference usage.

Validation evidence, open questions, unknowns, history, roadmaps, checklists, and next actions belong outside Markdown.

## Key documentation

- [Control Architecture](docs/architecture/control_architecture.md)
- [Control Contracts](docs/architecture/control_contracts.md)
- [Fault Registry](docs/architecture/fault_registry.md)
- [Observe-only State Safety](docs/architecture/observe_only_state_safety.md)
- [Runtime Profiles](docs/architecture/runtime_profiles.md)
- [Communication and Parameter Architecture](docs/architecture/communications.md)
- [Telemetry Schema](docs/architecture/telemetry_schema.md)
- [Firmware Runtime](docs/commissioning/firmware-runtime.md)
- [Motor Characterization Interface](docs/commissioning/motor-characterization-interface.md)
- [Controller Implementation](docs/control/controller-implementation.md)
- [Repository Layout](docs/development/repository-layout.md)
- [Build and Test](docs/development/build-and-test.md)
- [Forest D1 2016 Hardware Baseline](docs/hardware/forest-d1-2016-baseline.md)
