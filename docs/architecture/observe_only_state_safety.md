# Observe-only State Safety Profile

The default application control profile is diagnostic and structurally configured.

## Values

| Field | Value |
|---|---:|
| `configured` | `true` |
| `max_sample_age_us` | `0` |
| max pendulum angle | `FLT_MAX` |
| max arm angle | `FLT_MAX` |
| max pendulum rate | `FLT_MAX` |
| max arm rate | `FLT_MAX` |
| `motor_output_enabled` | `false` |
| automatic motor sink | unbound |

Argument, sensor-validity, estimator-readiness, finite-state, and timestamp-consistency checks remain active. Physical angle/rate limits are non-restrictive in this profile.

With `motor_output_enabled = false` and the automatic sink unbound, the profile cannot actuate the motor through the automatic-control path.
