# Communication and Parameter Architecture

## Current transport

Text UART maintenance transport is implemented. The control and sensor core remain independent of transport-specific types.

Micro XRCE-DDS is **not implemented**. COBS transport is **not implemented**.

## Current maintenance behavior

The current firmware exposes text commands for status, telemetry, runtime parameters, and bounded maintenance motor control.

`param set` changes active RAM values. Persistent `param save` / `param load` storage is not implemented.

The maintenance motor arm window is **30 seconds**. Ordinary `motor test` accepts signed duty magnitude from 1% through 20% and duration from 50 ms through 10000 ms. Stop/completion paths return the interface to stopped/disarmed state.

## Runtime parameters

| Name | Current default | Range / meaning |
|---|---:|---|
| `sensor.pendulum.upright_adc` | `2928` | Provisional current upright ADC reference |
| `sensor.pendulum.direction` | `1` | `-1` or `1`; angle sign convention |
| `telem.rate_hz` | `10` | Text telemetry rate, 1..20 Hz |

The pendulum conversion wraps the raw ADC delta before scaling to the circular angle domain.

## Physical authority

The text motor commands are maintenance operations. They do not grant automatic closed-loop controller authority.
