# Runtime Validation - 2026-08-15

## Baseline

```text
commit: b6bd30d414c906cf9de87e9110499834cc749663
firmware: 0.1.0-dev
MCU: STM32F103C8T6
clock: 72 MHz
control cadence: 1 kHz
profile: observe-only
motor sink: unbound
```

This record is the pre-P1 runtime regression baseline.

## Build validation

`tools/post_patch_check.sh` completed successfully:

```text
host build: PASS
host tests: 24/24 PASS
STM32 configure: PASS
STM32 build: PASS
ROM: 52936 B / 64 KB = 80.77%
RAM: 3428 B / 20 KB = 16.74%
text: 52820 B
data: 116 B
bss: 3312 B
```

Firmware artifact SHA256 values:

```text
ELF d46853a9eece080ab639a89fd2d9865703653c404e68ddb54c6bc727d6885701
HEX eb513bc6cb12c8df06d57077c316f43c84bcd28155c2cbbd42886eb2f1572e81
BIN 02ea93dd705167cfbd0cebcacebc1f7b93a2dcad31b99640f8bd7361fe8ed2d7
```

## Boot/runtime contract

Observed boot state:

```text
[CONTROL] runtime=ready profile=observe-only motor_sink=unbound arm_home=boot-position
[CONTROL_GATE] mode=observe-only operator=disabled config=ready sample_age_us<=2000 entry_theta_mrad<=250 allowed=0 motor_sink=unbound
[MOTOR_AUTH] owner=none maintenance=arbiter control=unbound gate=disabled watchdog=5ms fault=latching
```

This confirms that automatic closed-loop control remains unable to drive the physical motor.

## Performance baseline

Representative 5-second windows remained stable around:

```text
ctrl_avg_us=143
ctrl_max_us=145..148
work_avg_us=168
work_max_us=693..697
cpu_sched_pct=16.8
sample_age_us=25
missed approximately 0..1 per 5-second report window
```

Telemetry foreground work remained approximately 1 us average, with observed maximum 17 us while telemetry was enabled. OLED foreground work remained approximately 6 us average with maxima around 531..534 us.

## State-safety observation

Normal samples reported:

```text
faults=0x00000002
mode=disabled
allowed=0
gate_allowed=0
gate_reject=0x0000005A
```

`0x00000002` is `STATE_SAFETY_FAULT_CONFIG`: the state-safety limits are intentionally still unconfigured in the observe-only STM32 startup path.

One sample reported:

```text
faults=0x0000000A
```

which decodes as:

```text
STATE_SAFETY_FAULT_CONFIG
STATE_SAFETY_FAULT_ESTIMATE_NOT_READY
```

That sample coincided with a large pendulum-input / estimated-rate transient and immediately returned to the normal configuration-only mask. Preserve this event as future record/replay and estimator-robustness evidence.

## Gate interpretation

With the operator request disabled, the observed gate reject mask remained:

```text
0x5A = OPERATOR + MODE + STATE_SAFETY + ENTRY_ANGLE
```

Earlier validation with `control.enable_request=1` removed only the operator rejection and produced `0x58`, confirming that operator-request propagation works independently of the remaining admission conditions.

## Conclusion

Status: PASS as a pre-P1 regression baseline.

No regression was observed in build, host tests, boot contracts, timing, UART telemetry, motor-authority ownership, or observe-only closed-loop isolation.

## Next action

P1 should:

1. expose control safety/status through a dedicated runtime accessor rather than depending on optional trace capture;
2. guarantee a fresh fail-closed gate result when runtime/control data is unavailable;
3. add an explicit observe-only state-safety configuration instead of bypassing `STATE_SAFETY_FAULT_CONFIG`;
4. keep `motor_output_enabled=false` and the automatic-control motor sink unbound.
