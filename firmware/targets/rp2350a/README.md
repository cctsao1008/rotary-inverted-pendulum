# RP2350A Pico SDK target

This target replaces the STM32F103 execution platform with the UNO-form-factor RP2350A board while preserving the production/runtime semantics already implemented on `main`.

The implementation uses Raspberry Pi Pico SDK and restrained C++17 (`-fno-exceptions`, `-fno-rtti`). It is a platform port, not a second control architecture.

## Functional scope

The target implements the complete path needed before physical commissioning:

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
command safety / runtime authority
        ↓
TB6612 electrical mapping
        ↓
RP2350 PWM + direction backend
```

The physical TB6612 backend is complete, including break-before-make direction changes and unconditional safe-off. Automatic physical authority is **not** requested at boot, matching the current STM32 `main` policy: sensing, estimation, controller computation and evidence run while the motor remains uncommanded unless Supervisor authority is explicitly added later.

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

The encoder pins are non-consecutive. The stock Pico SDK PIO quadrature example requires two consecutive sampled pins, so this target uses both-edge GPIO IRQ quadrature decoding. This preserves the frozen UNO-shield wiring without inventing an unnecessary custom PIO program.

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

## USB

USB is a TinyUSB composite device:

```text
USB
├── CDC ACM
│   └── debug / commissioning console
└── vendor HID (64-byte report)
    └── deterministic runtime telemetry snapshot
```

CDC commands:

```text
help
version
status
telemetry on
telemetry off
```

HID telemetry defaults off, preserving the existing assembly policy, and is explicitly enabled by the host. The HID report includes raw ADC, encoder count, estimated state, regime, torque demand, bounded command and timing evidence.

Unsolicited CDC debug output can be disabled without removing the CDC console:

```bash
cmake -S firmware/targets/rp2350a -B build/rp2350a \
  -DRIP_ENABLE_CDC_LOG=OFF
```

## Board definition

`boards/uno_rp2350.h` records:

- RP2350A package;
- external Winbond W25Q128JVSIQ QSPI flash;
- 16 MiB flash geometry.

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

CI also compiles and executes native C++ semantic checks for measurement mapping, circular-angle estimation, hybrid regime transition, actuator mapping and command-safety slew behavior before building the RP2350 image.

## Physical commissioning follows software parity

The finished firmware is intended to be present before physical characterization. Hardware commissioning will then determine specimen-specific facts rather than gate implementation:

1. verify shield power domains and RP2350 logic compatibility;
2. read and calibrate the physical pendulum ADC range;
3. verify encoder direction/count scale and measure arm position/velocity;
4. characterize motor sign, dead zone, speed response and position response;
5. promote bounded physical motor authority only after those results justify it.

The three known electrical checks remain mandatory before powered integration: encoder output HIGH level, shield `VCC50` interaction with the UNO power header, and TB6612 recognition of 3.3 V RP2350 logic HIGH.
