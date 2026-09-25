# RP2350 Commissioning Tool

This folder owns the host-side test and commissioning interface for the RP2350A target. CDC and HID are managed by one tool and one session:

```text
rp2350_commission.py
        |
        +-- HID: binary telemetry + test commands
        +-- CDC: debug/status console + log capture
```

The user-facing entry point is always:

```bash
python tools/rp2350_commission/rp2350_commission.py <command>
```

Install dependencies with:

```bash
python -m pip install -r tools/rp2350_commission/requirements.txt
```

## Commands

```text
status
monitor
adc
encoder
motor-direction
speed-sweep
position-step
step-response
chirp
prbs
all
```

`all` runs status, ADC, encoder, motor direction, speed sweep, position step, open-loop step response, chirp, and PRBS in that order. Passive checks run first; one confirmation is required before the active portion unless `--yes` is supplied.

## Communication split

- **HID** carries 100 Hz machine-readable telemetry and acknowledged test commands.
- **CDC** carries human-readable `help`, `version`, `status`, debug/event text, and captured logs.

HID commands:

```text
GET_STATUS
TELEMETRY_ON
TELEMETRY_OFF
SET_MOTOR_COMMAND
SAFE_OFF
```

Motor test commands are intentionally direct. Firmware only checks the normalized command range and keeps a short timeout so a stopped host does not leave a stale command active. There is no extra arm/maintenance handshake and no firmware-side slew limiter to distort step, chirp, or PRBS tests.

Current command range is the full normalized interval `[-1.0, +1.0]`. The tool's default tests remain much smaller unless explicitly changed.

## Evidence

Every recorded test writes to:

```text
artifacts/commissioning/<timestamp>-<test>/
    metadata.json
    samples.csv
    cdc.log
    summary.json
    result.md
```

HID telemetry is the machine-facing measurement record. CDC is captured alongside it as diagnostic/event evidence.

## Current RP2350 signal mapping

```text
Motor A
  PWM   D10 / GPIO10
  AIN1  D13 / GPIO13
  AIN2  D12 / GPIO12

Encoder1
  A     D9 / GPIO9
  B     D2 / GPIO2

Pendulum
  ADC   A0 / GPIO26 / ADC0
```

Telemetry includes raw Encoder1 A/B states, accumulated encoder count, ADC raw value, estimated state, runtime timing evidence, controller demand, and the command actually applied to the motor backend.

## Test intent

- `adc`: raw range/noise evidence for pendulum calibration.
- `encoder`: live A/B states, accumulated count and arm state.
- `motor-direction`: positive/negative command sign versus encoder count direction.
- `speed-sweep`: normalized command versus steady arm velocity.
- `position-step`: host-side PD position characterization.
- `step-response`: open-loop step evidence.
- `chirp`: swept-sine excitation for frequency-domain/SysID work.
- `prbs`: deterministic seeded excitation for SysID.
