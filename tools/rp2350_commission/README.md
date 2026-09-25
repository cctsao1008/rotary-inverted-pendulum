# RP2350 Commissioning Tool

One host-side tool owns RP2350A testing over HID + CDC.

```bash
python tools/rp2350_commission/rp2350_commission.py <command>
```

Commands:

```text
status
safe-off
monitor
adc
encoder
motor
motor-direction
speed-sweep
position-step
step-response
chirp
prbs
all
```

Examples:

```bash
python tools/rp2350_commission/rp2350_commission.py status
python tools/rp2350_commission/rp2350_commission.py monitor --duration 10
python tools/rp2350_commission/rp2350_commission.py motor --command 0.15 --duration 2
python tools/rp2350_commission/rp2350_commission.py speed-sweep
python tools/rp2350_commission/rp2350_commission.py all
```

HID commands are deliberately small: `GET_STATUS`, `TELEMETRY_ON`, `TELEMETRY_OFF`, `SET_MOTOR_COMMAND`, and `SAFE_OFF`.

`SET_MOTOR_COMMAND` directly controls normalized motor command. Firmware checks `[-1.0, +1.0]` and expires stale commands after a short timeout. There is no extra test-mode handshake, arming sequence, maintenance-authority layer, or firmware slew limiter.

CDC carries debug/status/log text. HID carries commands and 100 Hz binary telemetry. Telemetry includes Encoder1 A/B, accumulated count, ADC raw value, estimated state, timing evidence, and applied motor command.

All test commands run immediately when invoked; there are no interactive confirmation prompts. Active tests call `SAFE_OFF` when they finish, and the firmware timeout stops a stale command if the host disappears.

Recorded tests write under:

```text
artifacts/commissioning/<timestamp>-<test>/
```

with raw samples and summary evidence for later SysID and controller work.
