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
- **Control** — desired closed-loop behavior: energy-based swing-up, capture/transition management, and LQR balance.
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
Firmware ActuationSink
    ↓
Tb6612ElectricalActuation
    ↓
Tb6612FrameIo
    ↓
STM32F103 PWM / GPIO physical output
```

`RawObservation` is Plant-owned observation semantics populated by Firmware. `EstimatorMeasurement` is a Supervisor-owned estimator input representation that preserves Plant measurement semantics. State estimation belongs to Supervisor.

The STM32F103 executable materializes the non-actuating computation path through the authority decision:

```text
TIM1 / 1 kHz control opportunity
    ↓
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
HybridController
    ├── EnergySwingUpController
    ├── CapturePolicy + blended transition
    └── LqrController
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

TIM1 provides a fixed 1 kHz control opportunity. The hardware update flag is not treated as a backlog queue: one observed update admits at most one fresh acquisition/control cycle, and missed periods are not replayed. DWT/`MonoTimer` remains the independent monotonic timing source used for timing evidence.

The target includes on-target runtime timing characterization. Debugger-visible counters expose admitted-cycle count and elapsed time, last/min/max admission period, maximum period jitter relative to the nearest 1 kHz slot, maximum TIM1 admission phase, last/max end-to-end execution time, maximum execution cycles, inferred coalesced/missed ticks, deadline overruns, Supervisor late/timeout counts, ADC errors, and runtime errors. Execution timing covers the admitted sensing → estimation → hybrid control → actuator-model → authority path and its shadow publication.

The target also enables the STM32 independent watchdog (`IWDG`) with a 100 ms timeout. It is fed after each admitted opportunity is serviced, so a stalled firmware loop resets the MCU independently of TIM1 and DWT. Boot re-establishes the D2 motor channel in hard safe-off.

`RuntimeAuthority` remains disarmed and the runtime remains `Ready`, so this executable cannot produce closed-loop `AuthorizedActuation`.

The installed D2 motor channel is concretely bound in a hard safe-off state at boot: PB1/TIM3_CH4 is configured for 20 kHz PWM with zero duty, and PB13/PB12 are driven low. No runtime `ActuationSink` owns these peripherals, so control computation cannot reach the physical motor.

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

`plant/actuator-model` provides both a compact static torque-span model and a speed-aware DC-motor model with armature resistance, torque constant, back-EMF, gearbox ratio/efficiency, supply-voltage authority, command deadzone, and explicit current limiting.

## Hybrid control

The Control domain implements three regimes:

```text
SwingUp
   ↓ capture window
Capture
   ↓ settled upright window
Balance
```

`EnergySwingUpController` implements an energy-balance law and a bounded dead-start kick. `CapturePolicy` provides separate entry/exit thresholds, settle-cycle qualification, and hysteresis. During `Capture`, swing-up and LQR torque demands are blended as the pendulum approaches the balance region. `Balance` uses LQR state feedback.

Operational permission remains separate from control regime:

```text
RuntimeState                 ControlRegime
-----------                  -------------
Disabled                     SwingUp
Ready                        Capture
Active(ControlRegime)        Balance
Fault(reason)
```

`Ready` may execute non-actuating live-shadow computation. Closed-loop physical authorization still requires `RuntimeState::Active(...)` plus the remaining Supervisor authority conditions.

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
        ↓
actuator-specific frame
        ↓
frame I/O backend
        ↓
physical output
```

`AuthorizedActuation` has no public constructor. Maintenance output uses a distinct `MaintenanceActuation` proof type and a mutually exclusive Supervisor-owned maintenance authority mode.

`firmware/actuators/tb6612` separates three roles:

```text
AuthorizedActuation / MaintenanceActuation
        ↓
Tb6612Output               Firmware ActuationSink
        ↓
Tb6612Mapper               authority-proof -> electrical semantics
        ↓
Tb6612ElectricalActuation  actuator-specific frame
        ↓
Tb6612FrameIo              target/backend boundary
```

Drive frames cannot be publicly constructed. `Tb6612Output` owns the frame backend and exposes no public arbitrary-frame application route; its closed-loop and maintenance entry points require their respective Supervisor proof types. `safe_off()` remains the only unqualified output action.

`Tb6612PwmDirIo` is the generic PWM/direction backend implementation and performs break-before-make: PWM is forced to zero before direction pins change, then the requested duty is applied. A target may provide another `Tb6612FrameIo` implementation without changing the authority semantics.

The STM32F103 executable still does not instantiate a runtime `Tb6612Output` or hand its concrete D2 motor peripherals to an actuation sink.

## Reference-backed nominal live-shadow parameters

The STM32F103 live-shadow controller uses a QNET rotary-inverted-pendulum reference model from Abdullah et al. (2021), not Forest D1 specimen calibration.

The published voltage-domain LQR gain vector and DC-motor constants are converted to the project state order and rotary-arm torque output:

```text
project state order: [theta, theta_dot, phi, phi_dot]
nominal torque-feedback gains:
[0.18355, 0.01585, 0.01120, 0.00766]
```

The live-shadow swing-up model uses the same reference family for pendulum mass, center-of-mass length, inertia, target energy, and energy-balance gain. These values define a reference-backed computation path and are not claims of Forest D1 specimen calibration.

## Source ownership

```text
plant/
├── robot-domain/             Physical state, units, and generalized demand
├── plant-observation/        Raw observation semantics
├── measurement-model/        ADC/encoder measurement physics
└── actuator-model/           Static and speed-aware actuator models

control/
├── state-feedback/           Controller contract and LQR
└── hybrid-control/           Energy swing-up, capture policy, LQR transition

supervisor/
├── state-estimator/          Estimator input contract and state estimation
├── runtime-state/            Runtime policy, timing, watchdog, authority
└── control-runtime/          Deterministic portable control composition

firmware/
├── interfaces/actuation/     Authorized physical-output contract
├── actuators/tb6612/         TB6612 mapper, proof-gated sink, frame-I/O boundary
├── adapters/estimator-input/ Raw observation -> estimator input promotion
└── targets/stm32f103/        1 kHz live-shadow, timing characterization, IWDG, hard-safe-off D2 binding
```

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
