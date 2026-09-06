# 🌀 Rotary Inverted Pendulum

> **A ground-up re-architecture of a rotary inverted pendulum control system, from physical I/O to hybrid control.**

The implementation is Rust-first and `no_std`. The project shares one architectural grammar with `single-wheel-platform`: **same architecture, different plant**.

## Architecture

```text
                  CONTROL
                     ▲
                     │
                 SUPERVISOR
                  ▲      ▲
                  │      │
                PLANT    │
                  ▲      │
                  └──┬───┘
                     │
                 FIRMWARE
```

The four canonical domains are:

- **Plant** — physical state, units, measurement physics, actuator physics, and physical generalized input/output semantics.
- **Control** — desired closed-loop behavior such as LQR and hybrid control regimes.
- **Supervisor** — state estimation, runtime qualification, timing health, watchdogs, and physical-output authority.
- **Firmware** — sensor acquisition, electrical/protocol actuation semantics, board wiring, MCU peripherals, and executable target composition.

## Typed semantic path

```text
RawObservation
    ↓
EstimatorMeasurement
    ↓
EstimatedState
    ↓
GeneralizedDemand
    ↓
BoundedActuatorCommand
    ↓
AuthorizedActuation
    ↓
Tb6612ElectricalActuation
    ↓
STM32F103 PWM / GPIO physical output
```

`RawObservation` is Plant-owned observation semantics populated by Firmware. `EstimatorMeasurement` is a Supervisor-owned estimator input representation that preserves Plant measurement semantics. State estimation belongs to Supervisor.

The STM32F103 executable currently materializes a **non-actuating live-shadow prefix** through the authority decision:

```text
PA7 / ADC1 + PA0/PA1 / TIM2 + DWT timestamp
    ↓
RawObservation
    ↓
EstimatorInputAdapter
    ↓
EstimatorMeasurement
    ↓
BasicEstimator
    ↓
EstimatedState
    ↓
LqrController
    ↓
GeneralizedDemand
    ↓
ArmActuatorModel
    ↓
BoundedActuatorCommand
    ↓
RuntimeAuthority::evaluate
    ↓
AuthorityDecision + debugger-visible shadow data
```

`RuntimeAuthority` remains disarmed and the runtime remains `Ready`, so this target does not produce closed-loop `AuthorizedActuation`. It also does not link an `ActuationSink`, TB6612 electrical mapper, TIM3 motor PWM, or motor-direction GPIO backend.

## Rotary plant semantics

The estimated state is:

```text
x = [theta, theta_dot, phi, phi_dot]
```

where `theta` is the wrapped pendulum angle and `phi` is the continuous rotary-arm angle.

Control produces a physical generalized demand:

```text
EstimatedState
    ↓
Controller
    ↓
GeneralizedDemand { arm_torque }
```

The controller does not emit normalized PWM, direction GPIO, or H-bridge commands. Plant actuator modeling converts physical arm-torque demand into `BoundedActuatorCommand`. Firmware owns TB6612 electrical realization.

## Operational state and hybrid-control regime

Operational permission and control regime remain separate:

```text
RuntimeState                 ControlRegime
-----------                  -------------
Disabled                     SwingUp
Ready                        Capture
Active(ControlRegime)        Balance
Fault(reason)
```

`Ready` may execute non-actuating live-shadow computation. Closed-loop physical authorization still requires `RuntimeState::Active(...)` plus the other Supervisor authority conditions.

## Physical-output authority

Closed-loop physical output requires semantic promotion by Supervisor:

```text
BoundedActuatorCommand
        ↓
RuntimeAuthority
        ↓
AuthorizedActuation
        ↓
Firmware ActuationSink
```

`AuthorizedActuation` has no public constructor. Maintenance output uses a distinct `MaintenanceActuation` proof type and a mutually exclusive Supervisor-owned maintenance authority mode.

## Reference-backed nominal live-shadow parameters

The STM32F103 live-shadow controller currently uses a QNET rotary-inverted-pendulum reference model from Abdullah et al. (2021), not Forest D1 specimen calibration.

The published voltage-domain LQR gain vector and DC-motor constants are converted to the project state order and rotary-arm torque output:

```text
project state order: [theta, theta_dot, phi, phi_dot]
nominal torque-feedback gains:
[0.18355, 0.01585, 0.01120, 0.00766]
```

Using the reference values `Kt = 0.042 N·m/A`, `Rm = 8.4 Ω`, and the reported ±10 V LQR control saturation gives a zero-speed static nominal torque span of `0.05 N·m` for the shadow actuator model. These parameters define only the current reference-backed computation path; they do not grant or justify Forest D1 physical-output authority.

## Source ownership

```text
plant/
├── robot-domain/            Physical state, units, and generalized demand
├── plant-observation/       Raw observation semantics
├── measurement-model/       ADC/encoder measurement physics
└── actuator-model/          Demand -> bounded actuator command

control/
├── state-feedback/          Controller contract and LQR
└── hybrid-control/          SwingUp / Capture / Balance regimes

supervisor/
├── state-estimator/         Estimator input contract and state estimation
├── runtime-state/           Runtime policy, timing, watchdog, authority
└── control-runtime/         Deterministic portable control composition

firmware/
├── interfaces/actuation/    Authorized physical-output contract
├── actuators/tb6612/        TB6612 electrical semantics
├── adapters/estimator-input/ Raw observation -> estimator input promotion
└── targets/stm32f103/       STM32F103 sensing + live-shadow composition
```

The previous C implementation and superseded Rust architecture are retained in Git history rather than in the active tree.

## Reference-backed nominal parameters

Physical and model parameters may use **reference-backed nominal values** when a project-specific value is not part of the implemented system definition. A nominal parameter carries a value, unit, source, and applicability; it is not a claim of specimen-specific calibration.

## Documentation policy

Markdown describes only the resulting system: architecture, interfaces/contracts, implemented behavior/reference usage, and selected reference-backed nominal parameters. History, roadmaps, checklists, validation logs, and unresolved work belong outside project Markdown.

## Key documentation

- [Control Architecture](docs/architecture/control_architecture.md)
- [Runtime Supervisor](docs/architecture/runtime_supervisor.md)
- [Repository Layout](docs/development/repository-layout.md)
- [Build and Test](docs/development/build-and-test.md)
- [Forest D1 2016 Hardware Baseline](docs/hardware/forest-d1-2016-baseline.md)
