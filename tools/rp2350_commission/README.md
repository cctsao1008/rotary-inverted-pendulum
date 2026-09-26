# RP2350 Commissioning Tool

One host-side tool owns RP2350A testing over HID + CDC.

```bash
python tools/rp2350_commission/rp2350_commission.py <command>
```

Commands:

```text
status
safe-off
bootloader
led
neopixel
monitor
adc
free-swing
encoder
motor
motor-direction
breakaway
speed-sweep
coast-down
position-step
step-response
chirp
prbs
all
```

Examples:

```bash
python tools/rp2350_commission/rp2350_commission.py status
python tools/rp2350_commission/rp2350_commission.py led on
python tools/rp2350_commission/rp2350_commission.py led off
python tools/rp2350_commission/rp2350_commission.py neopixel red
python tools/rp2350_commission/rp2350_commission.py neopixel green
python tools/rp2350_commission/rp2350_commission.py neopixel blue
python tools/rp2350_commission/rp2350_commission.py neopixel white
python tools/rp2350_commission/rp2350_commission.py neopixel off
python tools/rp2350_commission/rp2350_commission.py bootloader
python tools/rp2350_commission/rp2350_commission.py monitor --duration 10
python tools/rp2350_commission/rp2350_commission.py monitor --motor-command 0.10 --duration 3
python tools/rp2350_commission/rp2350_commission.py motor --command 0.15 --duration 2
python tools/rp2350_commission/rp2350_commission.py encoder --motor-command 0.10 --duration 3
python tools/rp2350_commission/rp2350_commission.py free-swing --duration 10
python tools/rp2350_commission/rp2350_commission.py breakaway
python tools/rp2350_commission/rp2350_commission.py speed-sweep
python tools/rp2350_commission/rp2350_commission.py coast-down --command 0.20
python tools/rp2350_commission/rp2350_commission.py all
```

HID commands are deliberately small: `GET_STATUS`, `TELEMETRY_ON`, `TELEMETRY_OFF`, `SET_MOTOR_COMMAND`, `SAFE_OFF`, `SET_USER_LED`, `ENTER_USB_BOOTLOADER`, and `SET_NEOPIXEL`.

The active motor uses TB6612 channel B: D9/PWMB, D7/BIN1, D8/BIN2, with Encoder2 on D10/D4. The shield's J8 D6/D3 pins are generic PWM-capable breakouts; they are not the TB6612 PWMA/PWMB nets.

`SET_USER_LED` controls the UNO RP2350 onboard blue user LED on D13/GPIO13. D13 is also unused channel-A AIN1, while D10 is shared between PWMA and Encoder2_A. Firmware mirrors D12/AIN2 with D13/AIN1 during LED diagnostics so channel A remains in an equal-input, non-driving state.

`SET_NEOPIXEL` drives the onboard GPIO14 WS2812 through a dedicated 800 kHz PIO state machine. The diagnostic interface deliberately exposes only `off`, `red`, `green`, `blue`, and `white` at low brightness; its purpose is to verify the complete host HID → firmware command → PIO → WS2812 signal path, not to introduce a general RGB status framework.

`ENTER_USB_BOOTLOADER` clears any direct motor command, applies safe-off, returns an HID acknowledgement, then hands USB control to the RP2350 ROM bootloader. The ROM is entered with USB mass storage disabled and PICOBOOT enabled, so normal development updates do not mount an `RPI-RP2` drive.

The normal Windows firmware update entry point is:

```powershell
.\tools\rp2350_update.ps1
```

It triggers `ENTER_USB_BOOTLOADER` over HID, waits for PICOBOOT, uses the prebuilt `picotool` to run `load -u -v -x` on `build/rp2350a/rip_rp2350a.elf`, then waits for the application HID/CDC interfaces to return. Use `-Build` to build first:

```powershell
.\tools\rp2350_update.ps1 -Build
```

A board that predates `ENTER_USB_BOOTLOADER` support needs one final BOOTSEL/UF2 flash to bootstrap this capability. The BOOTSEL/UF2 path remains available for recovery afterward.

`SET_MOTOR_COMMAND` directly controls normalized motor command. Firmware checks `[-1.0, +1.0]` and expires stale commands after a short timeout. There is no extra test-mode handshake, arming sequence, maintenance-authority layer, or firmware slew limiter.

CDC carries debug/status/log text. HID carries commands and 100 Hz binary telemetry. Telemetry includes Encoder2 A/B, accumulated count, ADC raw value, estimated state, timing evidence, and applied motor command.

`encoder --motor-command ...` is the direct live encoder check: the tool rotates the arm while recording A/B, count, position, velocity, and applied command. `monitor --motor-command ...` prints the same live signals while directly driving the motor.

Characterization commands are host-side experiments; firmware only supplies primitive motor command and telemetry paths:

- `breakaway`: ramp normalized command in both directions and report the first level that produces encoder motion.
- `speed-sweep`: map normalized command to steady arm speed.
- `coast-down`: drive to speed, command zero/coast, and record the decay trace.
- `free-swing`: keep the motor off and record passive pendulum ADC motion. A period/frequency is only reported when the trace passes diagnostic validity gates: sufficient ADC excursion, excursion larger than sample-to-sample noise, Schmitt-style hysteretic crossings, enough telemetry samples per candidate period, at least two resolved periods, and consistent period spacing. Floating/noise-only input therefore returns `estimate_valid=false` and leaves the period/frequency null instead of manufacturing a frequency.
- `step-response`, `chirp`, and `prbs`: record bounded excitation evidence for SysID.

The free-swing validity gates are intentionally measurement-quality checks, not pendulum calibration or a plant-model assumption. Raw samples and the diagnostic fields (`estimate_reason`, ADC excursion, median step, hysteresis, crossing counts, and period consistency) are still recorded when no period is accepted.

`all` runs the characterization sequence directly, without interactive confirmation prompts. It uses a small `+0.10` motor command during encoder capture by default. Override it with `all --encoder-command <value>`; adjust passive pendulum capture with `--free-swing-duration` and ADC/encoder windows with `--sensor-duration`.

Active tests call `SAFE_OFF` when they finish, and the firmware timeout stops a stale command if the host disappears.

Recorded tests write under:

```text
artifacts/commissioning/<timestamp>-<test>/
```

with raw samples, CDC logs, metadata, and summary evidence for later SysID and controller work.
