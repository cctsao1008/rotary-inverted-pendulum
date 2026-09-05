# 🌀 Rotary Inverted Pendulum

> **A ground-up re-architecture of a rotary inverted pendulum control system, from physical I/O to hybrid control.**

The repository defines the current control-system architecture for a rotary inverted pendulum: hardware access, sensing, actuation, state estimation, control, mode ownership, telemetry, safety, and physical actuator authority are explicit boundaries.

The physical plant is the starting point. Controller algorithms are replaceable implementations inside the architecture.

## Current architecture

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

## Current implementation status

| Capability | Current state |
|---|---|
| STM32F103 platform | Implemented / buildable |
| Shared `platform/api/` contract | Implemented |
| RP2350 platform | Not implemented / not buildable |
| Basic state estimator | Implemented and runtime-selectable |
| LQR balance controller | Implemented and runtime-selectable |
| LQI | Stub; runtime-rejected |
| Energy swing-up | Stub; disabled and runtime-rejected |
| Capture controller | Stub; disabled and runtime-rejected |
| Kalman estimator | Not runtime-selectable |
| Automatic control motor sink | Unbound |
| Maintenance motor path | Implemented |
| Text UART maintenance transport | Implemented |
| Micro XRCE-DDS | Not implemented |
| COBS transport | Not implemented |

The current automatic-control runtime is observe-only. Physical motor access is provided only through the bounded maintenance path.

## Platform boundary

```text
platform/
├── api/            Shared board contract
├── stm32f103/      Current implementation
└── rp2350/         Reserved namespace; no supported implementation
```

The legacy Forest D1 / Forest S1 names are used only for original hardware and documentation provenance. They do not define the software architecture.

## Reference physical plant

- STM32F103C8T6 reference controller
- geared nominal 12 V rotary-arm DC motor
- quadrature Hall encoder
- conductive-plastic pendulum angular-position sensor
- TB6612FNG H-bridge
- original rotary inverted-pendulum mechanical plant

See [Forest D1 2016 Hardware Baseline](docs/hardware/forest-d1-2016-baseline.md) for the current hardware facts and remaining unknowns.

## Repository ownership

```text
app/                    Application integration and system orchestration
control/                Platform-independent estimation, control, safety, and mode logic
drivers/                Reusable device drivers
platform/api/           Shared hardware contract
platform/stm32f103/     STM32F103 implementation
platform/rp2350/        Unsupported RP2350 namespace
tests/                  Host-side deterministic tests
tools/                  Runtime and validation tooling
docs/architecture/      Current architecture and contracts
docs/commissioning/     Current commissioning interfaces and state
docs/control/           Current controller implementation status
docs/validation/        Current evidence semantics
docs/hardware/          Current hardware facts and unknowns
docs/development/       Current repository and build reference
```

## Documentation policy

Markdown files describe **current engineering truth only**. They may contain current architecture, interfaces, implementation status, measured results, operational usage, and explicit unknowns.

Change history, experiment journals, postmortems, roadmaps, checklists, and next actions belong in Git history, GitHub issues, or external artifacts rather than Markdown.

## Key documentation

- [Control Architecture](docs/architecture/control_architecture.md)
- [Control Contracts](docs/architecture/control_contracts.md)
- [Fault Registry](docs/architecture/fault_registry.md)
- [Observe-only State Safety](docs/architecture/observe_only_state_safety.md)
- [Runtime Profiles](docs/architecture/runtime_profiles.md)
- [Communication and Parameter Architecture](docs/architecture/communications.md)
- [Telemetry Schema](docs/architecture/telemetry_schema.md)
- [Current Firmware State](docs/commissioning/current-firmware-state.md)
- [Motor Characterization Interface](docs/commissioning/motor-characterization-interface.md)
- [Controller Status](docs/control/controller-strategy.md)
- [Validation and Evidence Model](docs/validation/evidence-model.md)
- [Repository Layout](docs/development/repository-layout.md)
- [Build and Test](docs/development/build-and-test.md)
- [Forest D1 2016 Hardware Baseline](docs/hardware/forest-d1-2016-baseline.md)
