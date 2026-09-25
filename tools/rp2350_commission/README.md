# RP2350 Commissioning Tool

This folder owns the host-side test and commissioning interface for the RP2350A target. CDC and HID are managed by one tool and one session:

```text
rp2350_commission.py
        |
        +-- HID: binary telemetry + test commands
        +-- CDC: debug/status console + log capture
```

Run:

```bash
python tools/rp2350_commission/rp2350_commission.py <command>
```

Install dependencies with:

```bash
python -m pip install -r tools/rp2350_commission/requirements.txt
```

Commands:

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

`all` runs the complete sequence. Passive checks run first; one confirmation is required before the active portion unless `--yes` is supplied.

## USB split

HID carries machine-readable telemetry and acknowledged test commands:

```text
GET_STATUS
TELEMETRY_ON
TELEMETRY_OFF
SET_MOTOR_COMMAND
SAFE_OFF
```

CDC carries human-readable `help`, `version`, `status`, debug/event text, and captured logs.

`SET_MOTOR_COMMAND` is deliberately direct. Firmware checks the normalized range `[-1.0, +1.0]` and applies a short stale-command timeout. There is no separate maintenance/authority handshake and no extra firmware slew limiter. The default tests use much smaller amplitudes unless explicitly changed.

## Evidence

Each recorded test writes:

```text
artifacts/commissioning/<timestamp>-<test>/
    metadata.json
    samples.csv
    cdc.log
    summary.json
    result.md
```

## Signal mapping

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

Telemetry includes raw Encoder1 A/B states, accumulated count, ADC raw value, estimated state, timing evidence, and the command applied to the motor backend.
