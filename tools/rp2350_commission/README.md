# RP2350 Commissioning Tool

This folder owns the host-side commissioning interface for the RP2350A target. CDC and HID are intentionally managed by one tool and one session:

```text
rp2350_commission.py
        |
        +-- HID: machine-readable runtime telemetry and versioned commissioning protocol
        +-- CDC: human-readable debug/status console and log capture
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

Passive commands (`status`, `monitor`, `adc`, `encoder`) use the current CDC + HID firmware path directly. Active commands share the same CLI and evidence format, but they only run when firmware acknowledges the versioned HID commissioning command path. The tool fails closed rather than assuming that an ignored HID OUT report changed hardware state.

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

The telemetry report includes raw Encoder1 A/B states, accumulated encoder count, ADC raw value, estimated state, runtime timing evidence, controller demand, and command evidence.

## Safety boundary

Active tests are bounded commissioning operations, not automatic closed-loop admission. The CLI asks for explicit confirmation unless `--yes` is supplied. Firmware-side maintenance authority must independently enforce command magnitude, slew, safe-off, and a finite command lease. CDC remains diagnostic-only.
