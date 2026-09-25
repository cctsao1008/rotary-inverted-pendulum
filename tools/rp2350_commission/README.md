# RP2350 Commissioning Tool

One host-side tool owns RP2350A testing over HID + CDC.

```bash
python tools/rp2350_commission/rp2350_commission.py <command>
```

Commands: `status`, `monitor`, `adc`, `encoder`, `motor-direction`, `speed-sweep`, `position-step`, `step-response`, `chirp`, `prbs`, `all`.

HID commands: `GET_STATUS`, `TELEMETRY_ON`, `TELEMETRY_OFF`, `SET_MOTOR_COMMAND`, `SAFE_OFF`.

`SET_MOTOR_COMMAND` directly controls normalized motor command. Firmware checks `[-1.0, +1.0]` and expires stale commands after a short timeout. There is no extra test-mode handshake or slew limiter.

CDC carries debug/status/log text. Telemetry includes Encoder1 A/B, accumulated count, ADC raw value, estimated state, timing evidence, and applied motor command.
