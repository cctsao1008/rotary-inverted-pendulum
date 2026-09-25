# RP2350A Pico SDK target

This target is the hardware-commissioning firmware path for the UNO-form-factor RP2350A board used with the UNO Balance shield.

The implementation uses Raspberry Pi Pico SDK and restrained C++17. The existing Rust model, simulation, and STM32F103 work remain independent reference/legacy paths.

## Stage-0 scope

The first gate is intentionally small:

- build and boot on RP2350A;
- configure TB6612 channel-A command pins to safe idle immediately at startup;
- expose USB CDC for commissioning/debug output;
- encode the agreed board pin mapping;
- do **not** enable encoder acquisition, ADC acquisition, PWM authority, closed-loop control, or USB HID yet.

Expected CDC output is similar to:

```text
boot,target=rp2350a,board=uno_rp2350,safe_idle=1,motor_authority=0
status,safe_idle=1,motor_authority=0
```

## Canonical mapping

| Semantic signal | UNO shield | RP2350A GPIO |
| --- | --- | ---: |
| `ARM_MOTOR_PWM` | D10 / PWMA | 10 |
| `ARM_MOTOR_IN1` | D13 / AIN1 | 13 |
| `ARM_MOTOR_IN2` | D12 / AIN2 | 12 |
| `ARM_ENCODER_A` | D9 / ENCODER1_A | 9 |
| `ARM_ENCODER_B` | D2 / ENCODER1_B | 2 |
| `PENDULUM_ANGLE_ADC` | A0 / ADC0 | 26 |

Motor channel A is used (`MA+`, `MA-`, `ENCODER1_A`, `ENCODER1_B`). The pendulum potentiometer remains a 3.3 V ratiometric sensor.

## Board definition

`boards/uno_rp2350.h` records the board properties needed by Pico SDK:

- RP2350A package;
- external Winbond W25Q128JVSIQ QSPI flash;
- 16 MiB flash geometry.

The board schematic shows a 12 MHz crystal, which matches the RP-series default external crystal frequency expected by Pico SDK.

## Build

Use Raspberry Pi Pico SDK 2.3.1 or a compatible newer release and an Arm embedded GCC toolchain.

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
cmake -S firmware/targets/rp2350a -B build/rp2350a -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/rp2350a
```

The expected firmware artifacts include:

```text
build/rp2350a/rip_rp2350a.elf
build/rp2350a/rip_rp2350a.bin
build/rp2350a/rip_rp2350a.uf2
```

USB CDC logging is enabled by default. It can be disabled at configure time:

```bash
cmake -S firmware/targets/rp2350a -B build/rp2350a \
  -DRIP_ENABLE_CDC_LOG=OFF
```

## Commissioning blockers before sensor/motor bring-up

Do not progress from Stage 0 to bounded actuation until these electrical checks are resolved:

1. measure the motor encoder A/B HIGH level and verify it is safe for RP2350A GPIO;
2. verify the shield `VCC50` rail does not back-drive the RP2350 board 3.3 V rail through the UNO power header;
3. verify reliable TB6612 input-HIGH recognition from 3.3 V RP2350A outputs while the driver logic rail is at 5 V.

## Next gates

After Stage 0 passes on hardware, progress independently through pendulum ADC acquisition, PIO quadrature encoder acquisition, bounded TB6612 actuation, synchronized telemetry, motor characterization, passive pendulum dynamics, system identification, estimator integration, and finally control.
