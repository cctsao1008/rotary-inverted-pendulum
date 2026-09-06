# Communication and Parameter Architecture

## Transport

Text UART is the implemented maintenance transport. The control and sensor core remain independent of transport-specific types.

## Maintenance behavior

The firmware exposes text commands for status, telemetry, runtime parameters, and bounded maintenance motor control.

`param set` changes active RAM values.

The maintenance motor arm window is **30 seconds**. Ordinary `motor test` accepts signed duty magnitude from 1% through 20% and duration from 50 ms through 10000 ms. Stop/completion paths return the interface to stopped/disarmed state.

## Runtime parameters

| Name | Default | Range / meaning |
|---|---:|---|
| `sensor.pendulum.upright_adc` | `2928` | Compiled upright ADC reference |
| `sensor.pendulum.direction` | `1` | `-1` or `1`; angle sign convention |
| `telem.rate_hz` | `10` | Text telemetry rate, 1..20 Hz |

The pendulum conversion wraps the raw ADC delta before scaling to the circular angle domain.

## Reference-backed nominal parameters

Physical-plant and model parameters may be represented by reference-backed nominal values.

Accepted reference classes are:

- component datasheets and vendor hardware documentation;
- published rotary/Furuta-pendulum papers, theses, and technical reports;
- documented public implementations and experimental datasets for sufficiently comparable systems.

Each nominal parameter is defined by its value, unit, source, and applicability. A nominal value is a design/reference value rather than a specimen-specific calibrated measurement.

## Physical authority

Text motor commands are maintenance operations. They do not grant automatic closed-loop controller authority.
