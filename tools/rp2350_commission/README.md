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

HID commands are deliberately small: `GET_STATUS`, `TELEMETRY_ON`, `TELEMETRY_OFF`, `SET_MOTOR_COMMAND`, `SAFE_OFF`, `SET_USER_LED`, and `ENTER_USB_BOOTLOADER`.

`SET_USER_LED` directly controls the UNO RP2350 onboard blue user LED on D13/GPIO13. The active motor uses TB6612 channel B, while unused channel-A PWMA/D6 is held low, so toggling D13 cannot actuate motor channel A.

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
- `free-swing`: keep the motor off, record pendulum motion, and estimate the passive period/frequency from raw ADC crossings.
- `step-response`, `chirp`, and `prbs`: record bounded excitation evidence for SysID.

`all` runs the characterization sequence directly, without interactive confirmation prompts. It uses a small `+0.10` motor command during encoder capture by default. Override it with `all --encoder-command <value>`; adjust passive pendulum capture with `--free-swing-duration` and ADC/encoder windows with `--sensor-duration`.

Active tests call `SAFE_OFF` when they finish, and the firmware timeout stops a stale command if the host disappears.

Recorded tests write under:

```text
artifacts/commissioning/<timestamp>-<test>/
```

with raw samples, CDC logs, metadata, and summary evidence for later SysID and controller work.
