# Control Architecture

## Control-computation path

```text
Physical Sensors
    ↓
Sensor Acquisition
    ↓
Basic State Estimator
    ↓
State Safety
    ↓
Control State Machine
    ↓
Controller Dispatch
    ↓
Actuator Mapper
    ↓
Output Safety
    ↓
Computed Actuator Command
```

This path computes an actuator command. It does not implicitly own the physical motor.

## Physical actuation boundary

Automatic control has no physical motor sink:

```text
control_pipeline
    -> computed actuator command
    -> automatic motor sink = UNBOUND
```

The physical actuation path is maintenance-only:

```text
UART / maintenance command
    ↓
motor_test_service
    ↓
Motor Authority Arbiter (MAINTENANCE)
    ↓
board_motor
    ↓
TB6612
    ↓
Rotary-Arm Motor
```

The Motor Authority Arbiter is the physical ownership boundary. Controllers do not write PWM or direction GPIO directly.

## Ownership

| Component | Responsibility |
|---|---|
| `sensor_acquisition` | Raw measurements and timestamps |
| `state_estimator` | Control-domain state generation |
| `state_safety` | State validity and fail-closed control eligibility |
| `control_state_machine` | Control mode and controller-selection ownership |
| `controller_dispatch` | Dispatch to the implementation selected by mode |
| `actuator_mapper` | Abstract control effort to actuator-domain command |
| `output_safety` | Command-domain constraints and invalid-command rejection |
| `motor_authority` | Exclusive physical motor ownership |
| `board_motor` | MCU/H-bridge hardware implementation |

## Implemented control path

- Basic state estimator
- LQR balance-controller selection
- State-safety evaluation
- Control state machine
- Actuator mapping
- Output safety
- Observe-only automatic-control profile
- Bounded maintenance motor output

## State coordinates

- `theta`: pendulum angle; circular and wrapped for shortest-path angle differences.
- `theta_dot`: pendulum angular rate.
- `phi`: continuous accumulated rotary-arm angle relative to the active reference.
- `phi_dot`: rotary-arm angular rate.

The STM32 runtime uses firmware startup as the `phi = 0` reference.

## Safety and authority

State validity and physical authority are independent decisions. Invalid, stale, non-finite, unready, or faulted state denies control. Physical motor ownership is independently arbitrated by `motor_authority`.

The observe-only automatic-control profile keeps `motor_output_enabled = false` and the automatic motor sink unbound.
