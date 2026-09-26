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

The RP2350 target uses the UNO Balance J7/J8 interface with motor channel B and Encoder2. This keeps D13 available as the UNO RP2350 onboard user LED.

| Semantic signal | UNO shield / board | RP2350A GPIO |
| --- | --- | ---: |
| `ARM_MOTOR_PWM` | D3 / PWMB | 3 |
| `ARM_MOTOR_IN1` | D7 / BIN1 | 7 |
| `ARM_MOTOR_IN2` | D8 / BIN2 | 8 |
| `ARM_ENCODER_A` | D10 / ENCODER2_A | 10 |
| `ARM_ENCODER_B` | D4 / ENCODER2_B | 4 |
| `PENDULUM_ANGLE_ADC` | A0 / ADC0 | 26 |
| `USER_LED` | D13 / blue onboard LED | 13 |
| `NEOPIXEL` | onboard WS2812 data | 14 |

Motor channel B is used (`MB+`, `MB-`, `ENCODER2_A`, `ENCODER2_B`). The encoder pins are non-consecutive, so this target uses both-edge GPIO IRQ quadrature decoding.

Motor channel A is deliberately unused. Its PWMA input is D6; firmware holds D6 low from board initialization onward. D13 is physically shared with AIN1 on the shield, but with PWMA held low the onboard blue user LED can be driven without producing channel-A motor output. D12/AIN2 is also initialized low.

## Runtime and timing

- deterministic 1 kHz acquisition/control opportunity;
- monotonic microsecond timestamps from the RP2350 timer;
- missed opportunities coalesce instead of replaying as backlog;
- RP2350 hardware watchdog timeout is 100 ms;
- timing evidence is exposed to telemetry.

The mechanical geometry, 1040 count/rev arm scale, pendulum calibration, controller gains, and torque span remain the pre-commissioning baseline until mechanism-level validation. Two target-specific runtime parameters now use 2026-09-26 motor/encoder commissioning evidence: velocity filtering and the running-region command deadzone.

### Commissioned velocity filtering

The estimator still uses the production one-sample dirty-derivative structure, but no longer exposes raw 1 kHz quantization directly. At the current scales, one arm-encoder count per control tick is about `6.04 rad/s`, and one pendulum ADC count per tick is about `1.53 rad/s`. `kEstimatorRateFilterAlpha=0.10` gives the rate filter a roughly 9.5 ms time constant / 16.8 Hz pole while retaining substantially more bandwidth than the measured motor actuator.

This is a software correction to the rate estimator, not a claim that the pendulum sensor itself is fully commissioned. Pendulum ADC offset, direction, real installed noise, and dynamic response still require mechanism-level evidence.

### Commissioned friction handling

The unloaded speed sweep showed an approximately linear continuous-running region for `|command| >= 0.18`, with command-axis intercepts near `+0.065` and `-0.074`. The RP2350 target therefore uses a symmetric `0.07` kinetic command deadzone in the inverse actuator model.

Starting from rest was substantially more hysteretic and position-dependent than the running region. The most conservative observed positive breakaway was about `+0.23`, but other starts occurred at much smaller commands after recent motion or at different rotor/gear positions. That value is therefore retained as characterization evidence only; it is **not** hard-coded as a stationary automatic-control threshold yet.

This distinction is deliberate. A post-safety start gate could distort the existing slew-limit semantics, while a true stiction compensator would need state/history-aware behavior and should be tuned with the full mechanism installed. For now the runtime applies only the evidence-backed running-region deadzone and leaves static-start compensation for the mechanism-level commissioning pass.

These friction values are provisional actuator-side evidence. They do not replace the still-unvalidated torque/current model.

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

`SET_MOTOR_COMMAND` accepts a direct normalized command in `[-1.0, +1.0]`. Default characterization amplitudes now use the measured running region rather than the earlier `0.10` placeholder. The stale-command timeout is the only extra guard in this direct test path; there is no commissioning mode handshake or firmware slew limiter.

`SET_USER_LED` is a bare-board diagnostic command for the D13/GPIO13 blue onboard user LED. The host CLI exposes it directly:

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

Host entry points:

```bash
python tools/rp2350_commission/rp2350_commission.py <command>
python tools/rp2350_motor_suite.py
```

`rp2350_motor_suite.py` runs direction, breakaway detection, dense speed sweep, positive/negative coast-down, step, chirp, and PRBS characterization in one invocation. It preserves each section's raw evidence under `artifacts/commissioning/` and automatically derives count-slope speed fits, kinetic command intercepts, coast stop time/travel, step-response timing, and runtime timing statistics into the suite summary.

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

## Commissioning status

Completed RP2350 bare-board / motor-side evidence:

1. D13 user LED command path;
2. GPIO14 WS2812 red/green/blue/white/off path;
3. HID-to-PICOBOOT firmware update with flash verify and reboot;
4. floating-ADC false-frequency rejection;
5. Motor-B positive/negative actuation and Encoder2 quadrature/sign path;
6. breakaway detection and dense command-speed map;
7. positive/negative coast-down;
8. `±0.30` step response, 0.2–8 Hz chirp, and 200 ms PRBS;
9. 1 kHz runtime timing under active motor/encoder/USB load.

Still requiring mechanism-level evidence:

1. absolute 1040 count/rev confirmation against a known physical revolution;
2. installed pendulum ADC range, offset, sign, and noise;
3. passive pendulum dynamics / natural frequency;
4. coupled arm-pendulum system identification;
5. static-start/stiction compensation with the installed mechanism;
6. capture/balance controller tuning and closed-loop validation;
7. swing-up and transition validation.
