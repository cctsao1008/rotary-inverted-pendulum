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
TB6612 electrical mapping
        ↓
RP2350 PWM + direction backend
```

The production control path remains available for parity with `main`. Physical commissioning is deliberately simpler: the host can send a direct normalized motor test command over HID. Firmware only checks the normalized range and expires stale host commands after a short timeout.

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

The encoder pins are non-consecutive, so this target uses both-edge GPIO IRQ quadrature decoding.

## Runtime and timing

- deterministic 1 kHz acquisition/control opportunity;
- monotonic microsecond timestamps from the RP2350 timer;
- missed opportunities coalesce instead of replaying as backlog;
- RP2350 hardware watchdog timeout is 100 ms;
- timing evidence is exposed to telemetry.

Current calibration/controller constants intentionally preserve the STM32 live-shadow baseline until specimen commissioning, including the 1040 count/rev encoder scale and existing swing-up/capture/balance constants.

## USB and commissioning

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

HID telemetry is 100 Hz while the runtime remains 1 kHz. The report includes raw ADC, Encoder1 A/B logic states, accumulated encoder count, estimated state, applied motor command, and timing evidence.

`SET_MOTOR_COMMAND` accepts a direct normalized command in `[-1.0, +1.0]`. The default test amplitudes are much smaller. A short timeout remains so a stopped host does not leave an old command applied; there is no extra commissioning state machine or firmware slew limiter.

The host-side entry point is:

```bash
python tools/rp2350_commission/rp2350_commission.py <command>
```

## Build

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

## Physical commissioning

The finished image and unified tool are used to:

1. observe pendulum ADC raw range and calibration;
2. read Encoder1 A/B, count, arm position and velocity while the motor turns;
3. establish motor/encoder sign conventions;
4. characterize dead zone, speed and position response;
5. record step/chirp/PRBS data for SysID and later controller tuning.
