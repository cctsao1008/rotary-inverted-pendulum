# Observe-only State Safety Profile

The current default application control profile is diagnostic and structurally configured.

## Current values

| Field | Current value |
|---|---:|
| `configured` | `true` |
| `max_sample_age_us` | `0` |
| max pendulum angle | `FLT_MAX` |
| max arm angle | `FLT_MAX` |
| max pendulum rate | `FLT_MAX` |
| max arm rate | `FLT_MAX` |
| `motor_output_enabled` | `false` |
| automatic motor sink | unbound |

The profile preserves argument, sensor-validity, estimator-readiness, finite-state, and timestamp-consistency checks while making no claim that physical plant bounds are calibrated.

It is not an active closed-loop physical safety envelope.
