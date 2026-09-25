# RP2350 Commissioning Tool

One host-side tool owns RP2350A testing over HID + CDC.

```bash
python tools/rp2350_commission/rp2350_commission.py <command>
```

Commands:

```text
status  monitor  adc  encoder  motor-direction  speed-sweep
position-step  step-response  chirp  prbs  all
```

HID is the machine-facing path:

```text
GET_STATUS
TELEMETRY_ON
TELEMETRY_OFF
SET_MOTOR_COMMAND
SAFE_OFF
```

`SET_MOTOR_COMMAND` directly controls normalized motor command. Firmware checks `[-1.0, +1.0]` and expires stale commands after a short timeout. No extra test-mode handshake or slew limiter is involved.

CDC remains the human-readable debug/status/log path.

```text
Motor A:  D10/GPIO10 PWM, D13/GPIO13 AIN1, D12/GPIO12 AIN2
Encoder1: D9/GPIO9 A, D2/GPIO2 B
Pendulum: A0/GPIO26/ADC0
```

Telemetry includes Encoder1 A/B, accumulated count, ADC raw value, estimated state, timing evidence, and applied motor command. Test evidence is written under `artifacts/commissioning/<timestamp>-<test>/`.
