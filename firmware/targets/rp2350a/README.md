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

Motor channel A is used (`MA+`, `MA-`, `ENCODER1_A`, `ENCODER1_B`). The A0 row above is only the signal-routing definition. The pendulum sensor excitation voltage and maximum A0 voltage are **not** assumed safe for RP2350A until they are measured on the actual shield/mechanism.

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

The dedicated `RP2350 Pico SDK` GitHub Actions workflow also publishes the three Stage-0 outputs as the `rp2350a-stage0` artifact.

## Stage-0 hardware acceptance

Keep motor authority absent throughout this gate. The preferred bring-up sequence is:

1. **Bare RP2350A board, USB only:** flash `rip_rp2350a.uf2`, verify USB CDC enumeration, and observe `safe_idle=1,motor_authority=0`.
2. **Bare RP2350A board:** verify D10, D13, and D12 remain low after boot. No motor driver or mechanism is required for this check.
3. **Shield powered without the RP2350A board installed:** establish the real UNO power-header behavior before mating the boards. Measure shield `VCC50`, physical UNO `3.3V`, physical UNO `5V`, and `VIN`, and check continuity where needed.
4. **Shield + motor encoder, RP2350A board still absent:** measure `ENCODER1_A` and `ENCODER1_B` HIGH levels.
5. **Shield + pendulum sensor, RP2350A board still absent:** sweep the pendulum through its usable mechanical range and record A0 minimum and maximum voltage. Do not enable RP2350A ADC acquisition until the measured range is safe for the ADC input.
6. **Logic compatibility:** verify the TB6612 reliably recognizes a 3.3 V HIGH on its command inputs while its logic rail is at 5 V.
7. Only after the electrical checks pass, mate the RP2350A board and shield with the motor disconnected, power the system, and confirm USB CDC still reports Stage-0 safe idle.

Passing Stage 0 means the firmware boots reproducibly, USB CDC works, the motor command lines are electrically inactive, and the RP2350A/shield interface has no unresolved voltage-domain hazard. It does **not** grant motor authority.

## Commissioning blockers before sensor/motor bring-up

Do not progress from Stage 0 to sensor acquisition or bounded actuation until these electrical checks are resolved:

1. motor encoder A/B HIGH levels are safe for RP2350A GPIO;
2. shield `VCC50` does not back-drive or otherwise conflict with the RP2350A board power rails through the UNO header;
3. TB6612 input-HIGH recognition from 3.3 V RP2350A outputs is reliable with the driver logic rail at 5 V;
4. pendulum A0 voltage remains within a safe RP2350A ADC input range over the full intended mechanical travel.

## Next gates

After Stage 0 passes on hardware, progress independently through pendulum ADC acquisition, PIO quadrature encoder acquisition, bounded TB6612 actuation, synchronized telemetry, motor characterization, passive pendulum dynamics, system identification, estimator integration, and finally control.
