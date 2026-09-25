# RP2350 Commissioning Tool

This folder owns the host-side test interface for the RP2350A target. CDC and HID are managed by one tool and one session:

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

HID commands are deliberately minimal:

```text
GET_STATUS
TELEMETRY_ON
TELEMETRY_OFF
SET_MOTOR_COMMAND
SAFE_OFF
```

`SET_MOTOR_COMMAND` directly controls the normalized motor command. Firmware only checks `[-1.0, +1.0]` and uses a short stale-command timeout. There is no extra commissioning authority state, arm handshake, or slew limiter.

CDC carries human-readable `help`, `version`, `status`, debug/event text, and captured logs.

Each recorded test writes under `artifacts/commissioning/<timestamp>-<test>/` with CSV samples, CDC log, metadata, summary, and a short result report.

Current signal mapping:

```text
Motor A:  D10/GPIO10 PWM, D13/GPIO13 AIN1, D12/GPIO12 AIN2
Encoder1: D9/GPIO9 A, D2/GPIO2 B
Pendulum: A0/GPIO26/ADC0
```

Telemetry includes raw Encoder1 A/B states, accumulated count, ADC raw value, estimated state, timing evidence, and the applied motor command.
