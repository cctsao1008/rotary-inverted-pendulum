# RP2350A Pico SDK target

This target replaces the STM32F103 execution platform with the UNO-form-factor RP2350A board while preserving the production/runtime semantics already implemented on `main`.

The implementation uses Raspberry Pi Pico SDK and restrained C++17 (`-fno-exceptions`, `-fno-rtti`). It is a platform port, not a second control architecture.

## Functional scope

```text
Pendulum ADC + arm quadrature encoder + timestamp
        ↓
RawObservation
        ↓
measurement calibration / adaptation
        ↓
BasicEstimator
        ↓
EstimatedState [theta, theta_dot, phi, phi_dot]
        ↓
HybridController
   SwingUp → Capture → Balance
        ↓
GeneralizedDemand { arm_torque }
        ↓
ArmActuatorModel
        ↓
BoundedActuatorCommand
        ↓
production control qualification
        ↓
TB6612 electrical mapping
        ↓
RP2350 PWM + direction backend
```

The complete production-style control path remains available for parity with `main`. Physical commissioning is deliberately simpler: the host can send a direct normalized motor test command over HID, without a separate maintenance/authority handshake. Firmware only checks the normalized range and expires stale host commands after a short timeout.

## Canonical mapping

| Semantic signal | UNO shield | RP2350A GPIO |
| --- | --- | ---: |
| `ARM_MOTOR_PWM` | D10 / PWMA | 10 |
| `ARM_MOTOR_IN1` | D13 / AIN1 | 13 |
| `ARM_MOTOR_IN2` | D12 / AIN2 | 12 |
| `ARM_ENCODER_A` | D9 / ENCODER1_A | 9 |
| `ARM_ENCODER_B` | D2 / ENCODER1_B | 2 |
| `PENDULUM_ANGLE_ADC` | A0 / ADC0 | 26 |

Motor channel A is used (`MA+`, `MA-`, `ENCODER1_A`, `ENCODER1_B`).

The encoder pins are non-consecutive. The stock Pico SDK PIO quadrature example requires two consecutive sampled pins, so this target uses both-edge GPIO IRQ quadrature decoding.

## Runtime and timing

- deterministic 1 kHz acquisition/control opportunity;
- monotonic microsecond timestamps from the RP2350 timer;
- missed opportunities coalesce instead of replaying as backlog;
- software sensor-timing and control-watchdog semantics are retained;
- RP2350 hardware watchdog timeout is 100 ms;
- execution time, missed opportunities and deadline overrun evidence are exposed to telemetry.

Current calibration/controller constants intentionally preserve the STM32 live-shadow baseline until specimen commissioning:

- pendulum upright ADC: `2928`;
- pendulum scale: `2π / 4096` rad/count;
- arm encoder scale: `1040` counts/output revolution;
- estimator maximum gap: `20 ms`;
- 1 kHz nominal sampling;
- QNET-reference LQR gains: `[-0.18355, -0.01585, -0.01120, -0.00745]`;
- existing energy-swing-up and capture/balance thresholds from `main`.

These are software parity values, not claims that the RP2350 physical specimen is already calibrated.

## USB and commissioning

USB is a TinyUSB composite device:

```text
USB
├── CDC ACM
│   └── human-readable debug / status console
└── vendor HID (64-byte report)
    └── binary telemetry + test commands
```

CDC commands:

```text
help
version
status
telemetry on
telemetry off
```

HID commands:

```text
GET_STATUS
TELEMETRY_ON
TELEMETRY_OFF
SET_MOTOR_COMMAND
SAFE_OFF
```

HID telemetry is 100 Hz while the runtime remains 1 kHz. The report includes raw ADC, Encoder1 A/B logic states, accumulated encoder count, estimated state, regime, torque demand, applied command and timing evidence.

`SET_MOTOR_COMMAND` accepts a direct normalized command in `[-1.0, +1.0]`. The tool's default tests use substantially smaller commands. A short command timeout remains so a stopped host does not leave an old command applied; there is intentionally no additional firmware-side slew limiter because it would distort step/chirp/PRBS inputs.

The corresponding host-side entry point is:

```bash
python tools/rp2350_commission/rp2350_commission.py <command>
```

`tools/rp2350_commission/` keeps CDC/HID transport, protocol handling, passive sensor checks, motor characterization, position tests, SysID excitation and evidence recording in one folder while exposing one CLI.

Unsolicited CDC debug output can be disabled without removing the CDC console:

```bash
cmake -S firmware/targets/rp2350a -B build/rp2350a \
  -DRIP_ENABLE_CDC_LOG=OFF
```

## Board definition

`boards/uno_rp2350.h` records the RP2350A package, external Winbond W25Q128JVSIQ QSPI flash, and 16 MiB flash geometry.

## Build

Use Raspberry Pi Pico SDK 2.3.1 or a compatible newer release and an Arm embedded GCC toolchain.

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
cmake -S firmware/targets/rp2350a -B build/rp2350a -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/rp2350a
```

Expected artifacts:

```text
build/rp2350a/rip_rp2350a.elf
build/rp2350a/rip_rp2350a.bin
build/rp2350a/rip_rp2350a.uf2
```

CI compiles the host commissioning Python modules, executes native C++ semantic checks, and builds the RP2350 image.

## Physical commissioning

The finished image and unified tool are used to determine specimen-specific facts:

1. observe pendulum ADC raw range and establish specimen calibration;
2. rotate the arm while reading Encoder1 A/B, accumulated count, arm position and velocity;
3. establish motor/encoder sign conventions;
4. characterize motor dead zone, command-to-speed response and position response;
5. record step/chirp/PRBS evidence for SysID and later controller tuning.
