# RP2350 Commissioning Tool

One host-side tool owns RP2350A testing over HID + CDC.

```bash
python tools/rp2350_commission/rp2350_commission.py <command>
```

For motor/encoder characterization, prefer the one-shot suite instead of running each experiment manually:

```bash
python tools/rp2350_motor_suite.py
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
python tools/rp2350_commission/rp2350_commission.py monitor --motor-command 0.30 --duration 3
python tools/rp2350_commission/rp2350_commission.py motor --command 0.30 --duration 2
python tools/rp2350_commission/rp2350_commission.py encoder --motor-command 0.30 --duration 3
python tools/rp2350_commission/rp2350_commission.py free-swing --duration 10
python tools/rp2350_commission/rp2350_commission.py breakaway
python tools/rp2350_commission/rp2350_commission.py speed-sweep
python tools/rp2350_commission/rp2350_commission.py coast-down --command 0.40
python tools/rp2350_commission/rp2350_commission.py step-response --amplitude 0.30
python tools/rp2350_commission/rp2350_commission.py chirp --amplitude 0.30
python tools/rp2350_commission/rp2350_commission.py prbs --amplitude 0.30
python tools/rp2350_commission/rp2350_commission.py all
```

HID commands are deliberately small: `GET_STATUS`, `TELEMETRY_ON`, `TELEMETRY_OFF`, `SET_MOTOR_COMMAND`, `SAFE_OFF`, `SET_USER_LED`, `ENTER_USB_BOOTLOADER`, and `SET_NEOPIXEL`.

`SET_USER_LED` directly controls the UNO RP2350 onboard blue user LED on D13/GPIO13. The active motor uses TB6612 channel B, while unused channel-A PWMA/D6 is held low, so toggling D13 cannot actuate motor channel A.

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

## Motor-characterization baseline

The 2026-09-26 one-shot suite established these host-side defaults for this specimen:

- `motor-direction`: `±0.30`, well inside the repeatable running region.
- `speed-sweep`: dense points around the `0.18–0.25` transition plus `0.30/0.40/0.50` running points in both directions.
- `coast-down`: `±0.40` run-up.
- `step-response`, `chirp`, and `prbs`: `±0.30` excitation.

The earlier `0.10` excitation default is no longer used for SysID because commissioning showed that it lies inside the position-dependent stiction region and can intermittently move or remain stationary.

`breakaway` is intentionally a **first detectable motion** experiment. It is not a repeatable static-friction calibration: the observed threshold changed with rotor/gear position and recent motion. The firmware's automatic-control friction handling therefore uses separate running-region and stationary-start concepts instead of treating one breakaway result as a universal deadzone.

Characterization commands are host-side experiments; firmware only supplies primitive motor command and telemetry paths:

- `breakaway`: ramp normalized command in both directions and report the first level that produces detectable encoder motion.
- `speed-sweep`: map normalized command to steady arm speed.
- `coast-down`: drive to speed, command zero/coast, and record the decay trace.
- `free-swing`: keep the motor off and record passive pendulum ADC motion. A period/frequency is only reported when the trace passes diagnostic validity gates: sufficient ADC excursion, excursion larger than sample-to-sample noise, Schmitt-style hysteretic crossings, enough telemetry samples per candidate period, at least two resolved periods, and consistent period spacing. Floating/noise-only input therefore returns `estimate_valid=false` and leaves the period/frequency null instead of manufacturing a frequency.
- `step-response`, `chirp`, and `prbs`: record bounded excitation evidence for SysID.

The free-swing validity gates are intentionally measurement-quality checks, not pendulum calibration or a plant-model assumption. Raw samples and the diagnostic fields (`estimate_reason`, ADC excursion, median step, hysteresis, crossing counts, and period consistency) are still recorded when no period is accepted.

`rp2350_motor_suite.py` runs direction, breakaway, dense speed sweep, positive and negative coast-down, step response, chirp, PRBS, plus pre/post status in one invocation. A failure in one section is recorded and the suite continues after requesting `SAFE_OFF`; individual raw artifacts remain available for offline analysis.

`all` runs the broader commissioning sequence directly, without interactive confirmation prompts. Its encoder capture now defaults to `+0.30`; passive pendulum capture remains separate because the pendulum sensor/mechanism has not yet been commissioned. Override with `all --encoder-command <value>` and adjust windows with `--sensor-duration` / `--free-swing-duration`.

Active tests call `SAFE_OFF` when they finish, and the firmware timeout stops a stale command if the host disappears.

Recorded tests write under:

```text
artifacts/commissioning/<timestamp>-<test>/
```

with raw samples, CDC logs, metadata, and summary evidence for later SysID and controller work.
