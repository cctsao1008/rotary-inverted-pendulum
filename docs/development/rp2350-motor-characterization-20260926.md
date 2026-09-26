# RP2350 Motor / Encoder Characterization — 2026-09-26

This note records the motor-side evidence that changed the RP2350 target's estimator and actuator settings. It is intentionally limited to what was measured with the current Motor-B / Encoder2 setup. The pendulum mechanism and installed pendulum angle sensor were not part of this run.

## Test conditions

- RP2350A UNO-form-factor controller.
- UNO Balance shield, TB6612 channel B.
- Shield motor supply: 12 V.
- Motor-B command path: D3/PWMB, D7/BIN1, D8/BIN2.
- Encoder2: D10/A, D4/B.
- Runtime: 1 kHz.
- HID telemetry: 100 Hz.
- Arm conversion used the current provisional `1040 count/rev` constant. Therefore absolute `rad/s` values remain conditional on later physical CPR confirmation; command thresholds, direction, count-space timing, and command-axis intercepts do not depend on that scale.

The final one-shot suite used:

```text
motor direction       ±0.30
breakaway             0.01 command steps, max ±0.30
speed sweep           ±0.10, ±0.15, ±0.18, ±0.20, ±0.22,
                      ±0.25, ±0.30, ±0.40, ±0.50
coast-down            +0.40 and -0.40
step response         ±0.30
chirp                 ±0.30, 0.2 → 8 Hz, 20 s
PRBS                  ±0.30, 200 ms interval, 20 s
```

## Direction and quadrature

The direction run produced:

```text
+0.30 → +2073 encoder counts
-0.30 → -1878 encoder counts
```

All four quadrature states were previously observed during live rotation. The accepted convention is therefore:

```text
positive command → increasing Encoder2 count → positive phi
negative command → decreasing Encoder2 count → negative phi
```

## Start behavior is hysteretic

Breakaway was not repeatable as one scalar. Across commissioning runs, first detectable positive motion ranged from small creep to a cold/position-dependent case that did not clearly start until approximately `+0.23`. The one-shot suite itself reported first detectable motion at `+0.05` and `-0.10`, but these occurred after earlier motion and were only a few counts at the threshold.

The dense speed sweep also showed that `±0.10` may move in one run while `±0.15` can remain stationary in another segment. The correct interpretation is therefore:

- low command is dominated by static friction, gearbox state, rotor position, and history;
- `breakaway` is a first-detectable-motion experiment, not a universal deadzone calibration;
- automatic control should distinguish stationary start authority from already-moving friction compensation.

## Continuous-running region

Using encoder-count slope over the steady half of each speed-sweep segment, the repeatable running region was approximately linear for `|command| >= 0.18`.

Representative values with the provisional 1040 count/rev scale:

| Command | Positive speed (rad/s) | Negative speed magnitude (rad/s) |
| ---: | ---: | ---: |
| 0.18 | 6.74 | 5.74 |
| 0.20 | 7.38 | 6.66 |
| 0.22 | 8.45 | 7.46 |
| 0.25 | 10.56 | 9.94 |
| 0.30 | 13.04 | 12.27 |
| 0.40 | 18.85 | 17.83 |
| 0.50 | 24.44 | 22.81 |

Linear fits over `0.18…0.50` were:

```text
positive:  omega ≈ 56.19 * command - 3.68    R² ≈ 0.9991
negative: |omega| ≈ 54.10 * |command| - 4.03 R² ≈ 0.9980
```

The command-axis intercepts are approximately:

```text
positive ≈ 0.065
negative ≈ 0.074
```

Their symmetric midpoint, `0.07`, is used as the RP2350 target's provisional kinetic command deadzone. This is an empirical friction compensation term; it does **not** establish the still-unmeasured torque span.

## Coast-down

At `±0.40`, run-up speed was repeatable at roughly `18 rad/s` under the provisional scale. After electrical coast (`PWM=0`, `IN1=0`, `IN2=0`), the last observed encoder movement occurred after roughly `0.21–0.24 s` in both directions.

The decay was substantially closer to a near-linear speed decrease than to a pure exponential, consistent with gearbox / Coulomb-friction dominance over a simple viscous-only model.

This evidence supports keeping a separate stationary-start concept instead of representing all low-speed behavior with one viscous or deadzone parameter.

## Step, chirp, and PRBS

The `±0.30` step data showed motor-side rise dynamics on the order of tens of milliseconds. A simple first-order-plus-delay description fitted to the chirp fundamental was approximately:

```text
K  ≈ 37 (rad/s) / normalized command
τ  ≈ 27 ms
Td ≈ 12 ms
```

This is an actuator-side approximation only. It is not the coupled Furuta plant transfer function.

The 200 ms PRBS interval was long enough to expose direction-reversal transients while allowing the motor to approach the running-region speed before the next transition, so `±0.30 / 200 ms` remains a useful motor-side SysID stimulus.

## Rate-estimator implication

With the current provisional scales and a 1 kHz estimator:

```text
one arm encoder count / 1 ms ≈ 6.04 rad/s
one pendulum ADC count / 1 ms ≈ 1.53 rad/s
```

The previous `rate_filter_alpha = 1.0` exposed those derivative impulses directly. Motor telemetry therefore showed multi-rad/s apparent velocity ripple that was largely quantization, not real speed instability.

The RP2350 target now uses:

```text
rate_filter_alpha = 0.10
```

which corresponds to roughly a 9.5 ms discrete time constant / 16.8 Hz pole at the 1 kHz control rate. Unit tests exercise quantized encoder motion and one-count pendulum toggling so this correction remains explicit.

## Actuator-side software decision

The RP2350 automatic-control path now combines:

```text
kinetic command deadzone      = 0.07
stationary start floor        = 0.23
moving-rate threshold         = 0.50 rad/s
```

The `0.23` start value is deliberately conservative: it is the largest clearly observed positive start threshold in the commissioning sequence, not a universal plant constant. While the estimated arm is effectively stationary, automatic commands below this start floor are suppressed to zero. The gate only removes authority and therefore cannot defeat upstream command or slew limits.

Direct commissioning commands bypass the stationary gate so future characterization remains transparent.

## Runtime timing evidence

Across the uploaded characterization set, the 1 kHz runtime retained large headroom under active motor, encoder IRQ, USB, and telemetry load:

```text
missed opportunities = 0
deadline overruns     = 0
execution time median ≈ 13 us
execution time p99    ≈ 14 us
observed max          ≈ 23 us
```

## What remains unvalidated

This motor-side run does not establish:

- the physical 1040 count/rev scale against a known arm revolution;
- pendulum ADC installed offset, sign, range, or noise;
- pendulum natural frequency;
- coupled arm-pendulum parameters;
- the `0.05 Nm` actuator torque-span placeholder;
- LQR/capture performance;
- swing-up and capture transition behavior.

Those belong to the later mechanism-level commissioning pass.
