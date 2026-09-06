# Control Architecture

## Canonical domains

The system uses four architectural domains:

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

The arrows express ownership/dependency relationships, not runtime execution order.

## Plant

`plant/` owns portable physical truth:

- state and physical units;
- raw observation semantics;
- pendulum and encoder measurement physics;
- physical generalized input semantics;
- actuator capability/model constraints.

The Rotary estimated state is `[theta, theta_dot, phi, phi_dot]`. The generalized control input is rotary-arm torque.

## Control

`control/` owns desired closed-loop behavior.

```text
EstimatedState
    ↓
Controller
    ↓
GeneralizedDemand
```

The current state-feedback controller is LQR with state order `[theta, theta_dot, phi, phi_dot]`. LQR produces rotary-arm torque demand and does not own output saturation, PWM, GPIO, H-bridge semantics, runtime authority, or sensor acquisition.

`SwingUp`, `Capture`, and `Balance` are Control-domain hybrid-control regimes.

## Supervisor

`supervisor/` owns runtime belief, policy, health, and physical-output authority:

- `EstimatorMeasurement` input representation;
- `BasicEstimator` and `EstimatedState` production;
- sample freshness and runtime qualification;
- timing health and watchdog state;
- operating/runtime state;
- `RuntimeAuthority`;
- semantic promotion to `AuthorizedActuation`.

## Firmware

`firmware/` owns physical realization:

- sensor/target acquisition;
- actuator electrical/protocol semantics;
- physical-output interfaces;
- board/assembly binding;
- MCU-specific executable composition.

TB6612 direction/duty semantics are Firmware concerns. Concrete TIM3/GPIO realization belongs to the STM32F103 target backend.

## Typed semantic boundaries

```text
RawObservation
    != EstimatorMeasurement
    != EstimatedState
    != GeneralizedDemand
    != BoundedActuatorCommand
    != AuthorizedActuation
    != Tb6612ElectricalActuation
    != physical output
```

Each promotion has one owner. In particular, only Supervisor can create closed-loop `AuthorizedActuation`.
