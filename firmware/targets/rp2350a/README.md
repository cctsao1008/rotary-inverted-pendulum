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

The production control path remains available for parity with `main`. For physical testing, the host sends direct normalized motor commands over HID. Firmware only checks the normalized range and expires stale commands after a short timeout.

## Canonical mapping

The RP2350 target uses TB6612 motor channel B with Encoder2. The motor mapping must be read from the TB6612 U3 net labels, not inferred from the separate J8 PWM breakout: J8 exposes D6 and D3 as generic PWM-capable Arduino pins, but U3 PWMA/PWMB are D10/D9 respectively.

| Semantic signal | UNO shield / board | RP2350A GPIO |
| --- | --- | ---: |
| `ARM_MOTOR_PWM` | D9 / PWMB | 9 |
| `ARM_MOTOR_IN1` | D7 / BIN1 | 7 |
| `ARM_MOTOR_IN2` | D8 / BIN2 | 8 |
| `ARM_ENCODER_A` | D10 / ENCODER2_A | 10 |
| `ARM_ENCODER_B` | D4 / ENCODER2_B | 4 |
| `PENDULUM_ANGLE_ADC` | A0 / ADC0 | 26 |
| `USER_LED` | D13 / blue onboard LED | 13 |
| `NEOPIXEL` | onboard WS2812 data | 14 |

Motor channel B is used (`MB+`, `MB-`, `ENCODER2_A`, `ENCODER2_B`). The encoder pins are non-consecutive, so this target uses both-edge GPIO IRQ quadrature decoding.

Motor channel A is deliberately unused. The shield shares D10 between PWMA and ENCODER2_A, so PWMA cannot be forced low while Encoder2 is in use. Instead firmware keeps the channel-A direction inputs equal: D13/AIN1 and D12/AIN2 are both low in the normal safe state. `SET_USER_LED` mirrors D12 with D13, so channel A remains non-driving while the blue onboard LED is toggled.

## Runtime and timing

- deterministic 1 kHz acquisition/control opportunity;
- monotonic microsecond timestamps from the RP2350 timer;
- missed opportunities coalesce instead of replaying as backlog;
- RP2350 hardware watchdog timeout is 100 ms;
- timing evidence is exposed to telemetry.

Current calibration/controller constants intentionally preserve the STM32 live-shadow baseline until specimen commissioning, including the 1040 count/rev encoder scale and existing swing-up/capture/balance constants.

## USB and testing

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
SET_USER_LED
ENTER_USB_BOOTLOADER
SET_NEOPIXEL
```

HID telemetry is 100 Hz while the runtime remains 1 kHz. It includes raw ADC, Encoder2 A/B, accumulated count, estimated state, applied motor command, and timing evidence.

`SET_MOTOR_COMMAND` accepts a direct normalized command in `[-1.0, +1.0]`. Default test amplitudes are much smaller. The stale-command timeout is the only extra guard in this test path; there is no commissioning mode handshake or firmware slew limiter.

`SET_USER_LED` is a bare-board diagnostic command for the D13/GPIO13 blue onboard user LED. Because D13 is also channel-A AIN1, firmware mirrors D12/AIN2 to the same level so the unused channel-A bridge cannot command a direction while the LED changes:

```bash
python tools/rp2350_commission/rp2350_commission.py led on
python tools/rp2350_commission/rp2350_commission.py led off
```

`SET_NEOPIXEL` is a second bare-board diagnostic path for the onboard GPIO14 WS2812. Firmware uses one 800 kHz PIO state machine and exposes only dim `off`, `red`, `green`, `blue`, and `white` test colors:

```bash
python tools/rp2350_commission/rp2350_commission.py neopixel red
python tools/rp2350_commission/rp2350_commission.py neopixel green
python tools/rp2350_commission/rp2350_commission.py neopixel blue
python tools/rp2350_commission/rp2350_commission.py neopixel white
python tools/rp2350_commission/rp2350_commission.py neopixel off
```

`ENTER_USB_BOOTLOADER` first drives the motor path to safe-off, acknowledges the HID command, then reboots the RP2350 into its ROM USB bootloader with the mass-storage interface disabled and PICOBOOT left enabled. This is the normal development firmware-update path once the feature has been bootstrapped onto the board.

Host entry point:

```bash
python tools/rp2350_commission/rp2350_commission.py <command>
```

## Build and firmware update

On Windows, after Pico SDK and the prebuilt picotool package are available under `_deps`, use the thin helper scripts from the repository root:

```powershell
.\tools\rp2350_build.ps1
.\tools\rp2350_update.ps1
```

`rp2350_build.ps1` automatically uses `_deps/pico-sdk` when `PICO_SDK_PATH` is not already set, locates `picotoolConfig.cmake` under `_deps`, configures the Release Ninja build, builds the target, and verifies the ELF/BIN/UF2 outputs. Use `-Clean` when a fresh CMake configure is needed:

```powershell
.\tools\rp2350_build.ps1 -Clean
```

`rp2350_update.ps1` uses the running application HID interface to request a safe reboot into ROM PICOBOOT, waits for picotool access, programs only changed flash sectors from the ELF image, verifies the result, reboots the application, and waits for HID/CDC to return. Build and update can be combined:

```powershell
.\tools\rp2350_update.ps1 -Build
```

The legacy `rp2350_flash.ps1` BOOTSEL/UF2 path remains as a bootstrap and recovery mechanism. A board running firmware from before `ENTER_USB_BOOTLOADER` support needs one final BOOTSEL/UF2 flash to install the new updater-capable image; normal later updates do not require BOOTSEL or an `RPI-RP2` drive.

The equivalent manual build is:

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

The normal updater programs `rip_rp2350a.elf`; UF2 is retained for recovery.

## Physical tests

1. validate the onboard D13 user LED command on the bare RP2350 UNO board;
2. validate the onboard GPIO14 WS2812 red/green/blue/white/off command path;
3. observe pendulum ADC raw range and calibration;
4. read Encoder2 A/B, count, arm position and velocity while motor channel B turns;
5. establish motor/encoder sign conventions;
6. characterize dead zone, speed and position response;
7. record step/chirp/PRBS data for SysID and later controller tuning.
