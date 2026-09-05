# Control Contracts

## Coordinates

`theta` is the pendulum angle. It is circular and uses wrapped shortest-path deltas in the basic estimator.

`phi` is the continuous accumulated rotary-arm position relative to the active reference and is not wrapped at `+/-pi`. The STM32 sensor adapter extends encoder rollover before conversion to the control coordinate.

The runtime uses firmware startup as the `phi = 0` reference.

## Timing

`sample_period_s` defines the nominal control cadence. State derivatives use measured elapsed time between accepted samples.

A long forward timing gap re-seeds position history and clears rate readiness. Duplicate, backward, and otherwise invalid timestamps are rejected by the estimator/state-safety path.

## Configuration ownership

- Per-unit pendulum parameters are owned by `app/runtime_parameters`.
- Runtime control profile values are owned by `app/control_profile`.
- Estimator, controller, state-safety, and runtime structs form the platform-independent schema boundary.

## Runtime selection

The runtime configuration accepts:

- `STATE_ESTIMATOR_BASIC`
- `BALANCE_CONTROLLER_LQR`
- `swing_up_enabled = false`
- `capture_enabled = false`

## Observe-only state-safety contract

The application profile sets:

- `state_safety.configured = true`
- `max_sample_age_us = 0` in state safety
- pendulum/arm angle limits = `FLT_MAX`
- pendulum/arm rate limits = `FLT_MAX`
- `motor_output_enabled = false`
- automatic-control motor sink = unbound

The profile is diagnostic and cannot actuate the motor through the automatic-control path.

## Safety ownership

```text
state_safety
    -> control_state_machine
    -> actuator_mapper
    -> output_safety
    -> computed actuator command
```

Physical actuation is a separate authority chain through `motor_authority` and `board_motor`.

The maintenance motor path is the path that reaches the physical motor.
