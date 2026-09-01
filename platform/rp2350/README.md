# RP2350 Platform

## Status

**Planned / hardware pending validation.**

This directory defines the future RP2350 implementation boundary for the rotary inverted pendulum firmware. It does not yet represent a buildable or physically validated target.

The first planned hardware configuration combines:

- an Arduino UNO-form-factor RP2350A controller board
- a TB6612-based motor/encoder shield
- the existing rotary inverted pendulum plant
- a quadrature Hall encoder on the geared DC motor
- a conductive-plastic potentiometer for pendulum-angle measurement

## Intended peripheral mapping

The planned implementation direction is:

| Capability | RP2350 mechanism |
|---|---|
| Pendulum angle | ADC |
| Motor encoder | PIO-based quadrature capture |
| Motor command | Hardware PWM + direction GPIO |
| Timebase | RP2350 hardware timer |
| Host link | Native USB, initially CDC |
| Optional local logging | Deferred; must not perturb the control loop |

## Board-specific configuration

Concrete UNO/shield pin assignments belong under `boards/` once the physical hardware has been continuity-checked and voltage domains have been verified.

Schematic-derived mappings may be documented before arrival, but they must remain explicitly marked as unverified until confirmed on the assembled hardware.

## Enablement gates

Before this platform becomes a buildable target:

1. verify RP2350 board identity and Arduino-header GPIO mapping;
2. verify shield PWM, direction, encoder, power, and ADC wiring;
3. resolve the TB6612 logic-voltage margin for 3.3 V RP2350 GPIO;
4. confirm safe power ownership with USB and external motor supply present;
5. validate the pendulum potentiometer supply, range, zero, and polarity;
6. validate encoder direction and counts per revolution;
7. implement bounded motor actuation before any closed-loop authority is enabled.
