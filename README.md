# 🌀 Rotary Inverted Pendulum

> **A ground-up re-architecture of a rotary inverted pendulum control system, from physical I/O to hybrid control.**

The implementation is Rust-first and `no_std`. The architecture separates control mathematics, system operation, physical-plant conventions, and target-specific hardware integration.

## Architecture

```text
                    control
                       ▲
                       │
                   supervisor
                    ▲      ▲
                    │      │
                  plant    │
                    ▲      │
                    └──┬───┘
                       │
                    firmware
               STM32F103 / RP2350
```

The three primary architectural boundaries are:

- **Control Core** — state, estimation, control safety, control regime, controller implementations, and normalized control effort.
- **System Supervisor** — runtime state, observation cycle, operational policy, motor authority, and consumer-owned I/O ports.
- **Target Adapter** — MCU-specific startup, interrupts, peripherals, timing, and board integration.

`plant` is a reusable component library for physical conventions such as pendulum conversion, encoder scale, motor sign, and control-effort-to-drive mapping. It is not an additional architecture layer.

## Control semantics

The control state is:

```text
x = [theta, theta_dot, phi, phi_dot]
```

where `theta` is the wrapped pendulum angle and `phi` is the continuous rotary-arm angle.

Controllers produce `ControlEffort`. They do not produce PWM, direction GPIO, or H-bridge commands.

```text
ControlState
    ↓
Controller
    ↓
ControlEffort
```

Physical motor semantics are introduced outside the control core:

```text
ControlEffort
    ↓
Plant DriveMap
    ↓
DriveCommand
    ↓
Motor Authority
    ↓
MotorSink
```

## Operational state and control regime

Operational state and control regime are separate concepts.

```text
RuntimeState                 ControlRegime
-----------                  -------------
Disabled                     SwingUp
Ready                        Capture
Active(ControlRegime)        Balance
Fault(reason)
```

This prevents hardware/operational state from being conflated with hybrid-control mode transitions.

## Motor authority

`MotorAuthority` keeps a stable runtime type with dynamic authority state:

```text
Disarmed
Maintenance
Control
Fault
```

Physical output is available only through a `MaintenanceAccess` or `ControlAccess` capability returned by the authority boundary. The underlying motor sink remains private.

## Source ownership

```text
crates/control/          Pure control-domain computation
crates/supervisor/       Runtime supervision and physical authority
crates/plant/            Physical-plant conversions and drive conventions
firmware/stm32f103/      STM32F103 target composition
firmware/rp2350/         RP2350 target namespace when implemented
docs/architecture/       Architecture definition
docs/hardware/           Hardware definition and provenance
docs/development/        Repository/build reference
```

The previous C/CMake/libopencm3 implementation is retained in Git history rather than in the active source tree.

## Reference physical plant

- STM32F103C8T6 reference controller
- geared nominal 12 V rotary-arm DC motor
- quadrature Hall encoder
- conductive-plastic pendulum angular-position sensor
- TB6612FNG H-bridge
- original rotary inverted-pendulum mechanical plant

See [Forest D1 2016 Hardware Baseline](docs/hardware/forest-d1-2016-baseline.md).

## Reference-backed nominal parameters

Physical and model parameters may use **reference-backed nominal values** when a project-specific value is not part of the implemented system definition.

Preferred references are:

1. component datasheets and vendor hardware documentation;
2. published papers, theses, and technical reports for comparable rotary/Furuta pendulums;
3. documented public implementations and experimental datasets with sufficiently similar mechanics, sensing, actuation, or motor characteristics.

A reference-backed nominal parameter carries a value, unit, source, and applicability. **Nominal** means a representative engineering value; it is not a claim of specimen-specific calibration.

## Documentation policy

Markdown describes only the resulting system:

- architecture;
- interfaces and contracts;
- implemented behavior and reference usage;
- selected reference-backed nominal parameters.

Validation evidence, open questions, unknowns, history, roadmaps, checklists, and next actions belong outside Markdown.

## Key documentation

- [Control Architecture](docs/architecture/control_architecture.md)
- [Runtime Supervisor](docs/architecture/runtime_supervisor.md)
- [Repository Layout](docs/development/repository-layout.md)
- [Build and Test](docs/development/build-and-test.md)
- [Forest D1 2016 Hardware Baseline](docs/hardware/forest-d1-2016-baseline.md)
