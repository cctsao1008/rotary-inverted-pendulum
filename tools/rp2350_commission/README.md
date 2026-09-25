# RP2350 Commissioning Tool

This folder owns the host-side commissioning interface for the RP2350A target. CDC and HID are intentionally managed by one tool and one session:

```text
rp2350_commission.py
        |
        +-- HID: machine-readable runtime telemetry + acknowledged commissioning commands
        +-- CDC: human-readable debug/status console + log capture
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

`all` is the comprehensive suite: it runs status, ADC, encoder, motor direction, speed sweep, position step, open-loop step response, chirp, and PRBS in that order. Passive checks run first; one explicit operator confirmation is required before the active portion unless `--yes` is supplied.

Passive and active tests use the same session and evidence format. HID OUT commands are sequence-numbered and require a firmware acknowledgement; the host fails closed on timeout or rejection rather than assuming that an output report changed hardware state.

## Transport ownership

- **HID** owns machine-facing commissioning control and 100 Hz binary runtime telemetry.
- **CDC** owns human-facing `help`, `version`, `status`, debug/event text, and captured logs.
- CDC does not grant motor authority.

The commissioning HID commands are:

```text
GET_STATUS
TELEMETRY_ON
TELEMETRY_OFF
MAINTENANCE_ENTER
MAINTENANCE_EXIT
SET_MOTOR_COMMAND
SAFE_OFF
```

`SET_MOTOR_COMMAND` is only accepted while firmware is in explicit maintenance authority. The firmware independently bounds the normalized command, slew rate and finite command lease. If the lease is not refreshed, output returns to safe-off.

Current firmware commissioning limits:

```text
maximum |command|    0.50
maximum slew         2.0 command/s
default lease        250 ms
maximum lease        500 ms
```

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

The telemetry report includes raw Encoder1 A/B states, accumulated encoder count, ADC raw value, estimated state, runtime timing evidence, controller demand, applied command and authority state.

## Test intent

- `adc`: raw range/noise evidence for pendulum calibration.
- `encoder`: live A/B states, accumulated count and arm state.
- `motor-direction`: positive/negative command sign versus encoder count direction.
- `speed-sweep`: normalized command versus steady arm velocity.
- `position-step`: host-side bounded PD position characterization.
- `step-response`: bounded open-loop step evidence.
- `chirp`: bounded swept-sine excitation for frequency-domain/SysID work.
- `prbs`: deterministic seeded bounded excitation for SysID.

## Safety boundary

Active tests are commissioning operations, not automatic closed-loop admission. The CLI asks for explicit confirmation unless `--yes` is supplied. Firmware maintenance authority is distinct from closed-loop authority, and `SAFE_OFF` / lease expiry do not depend on the host continuing to run. The active path is therefore intentionally firmware-acknowledged rather than a host-only convention.
