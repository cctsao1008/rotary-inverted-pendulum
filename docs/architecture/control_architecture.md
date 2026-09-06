# Control Architecture

## Architectural boundaries

The software uses three primary boundaries rather than a traditional N-tier call stack.

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
```

Dependencies point toward the control core. Target-specific code must not define control-domain semantics.

## Control Core

`crates/control/` contains deterministic, target-independent control computation.

It owns:

- `ControlState` with state order `[theta, theta_dot, phi, phi_dot]`;
- `BasicEstimator`;
- control-state safety;
- `ControlRegime` (`SwingUp`, `Capture`, `Balance`);
- controller interfaces and LQR;
- bounded normalized `ControlEffort`.

The control core has no motor direction, PWM, GPIO, UART, interrupt, HAL, or MCU concepts.

```text
EstimatorInput
    ↓
BasicEstimator
    ↓
ControlState
    ↓
Control Safety
    ↓
Controller
    ↓
ControlEffort
```

## Plant Components

`crates/plant/` owns physical conventions that are reusable across MCU targets:

- pendulum ADC-to-angle conversion;
- encoder count-to-continuous-angle conversion;
- motor sign convention;
- normalized effort-to-drive mapping.

The resulting `DriveCommand` contains physical actuator semantics. These semantics do not enter the control core.

## System Supervisor

`crates/supervisor/` owns operational policy and runtime composition above the pure control computation.

It owns:

- operational runtime state;
- sensor observation ports;
- observe-only control-cycle orchestration;
- physical motor authority;
- motor and telemetry ports.

Ports are consumer-owned: the supervisor defines the capabilities it requires; firmware adapters implement those capabilities.

## Target Adapters

`firmware/<target>/` owns target-specific integration such as startup, interrupt wiring, clocks, ADC, encoder timers, PWM, UART, DMA, GPIO, and board composition.

Target adapters may depend on `supervisor`, `plant`, and `control`. The reverse dependency is not allowed.

## Execution planes

The runtime separates deterministic control work from background observability work.

```text
REAL-TIME CONTROL
observation -> estimate -> safety -> control -> qualify -> authority -> motor

BACKGROUND
commands / telemetry / OLED / diagnostics
```

Background work must be deferable and must not become a dependency of the control path.
