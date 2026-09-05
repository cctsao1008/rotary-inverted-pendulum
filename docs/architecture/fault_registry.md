# Fault Registry

The state-safety fault mask is defined by `control/state_safety.h`.

| Bit | Symbol | Meaning | Control consequence |
|---:|---|---|---|
| `0x00000001` | `STATE_SAFETY_FAULT_ARGUMENT` | Invalid/null safety input | Fail closed |
| `0x00000002` | `STATE_SAFETY_FAULT_CONFIG` | State-safety configuration invalid | Deny control |
| `0x00000004` | `STATE_SAFETY_FAULT_SENSOR_INVALID` | Sensor sample invalid | Fail closed |
| `0x00000008` | `STATE_SAFETY_FAULT_ESTIMATE_NOT_READY` | Estimator state not ready | Deny control |
| `0x00000010` | `STATE_SAFETY_FAULT_STATE_NONFINITE` | State contains non-finite value | Fail closed |
| `0x00000020` | `STATE_SAFETY_FAULT_TIMESTAMP` | State/sensor timestamp inconsistency | Fail closed |
| `0x00000040` | `STATE_SAFETY_FAULT_SAMPLE_TIMEOUT` | Sample exceeds configured age | Fail closed |
| `0x00000080` | `STATE_SAFETY_FAULT_PENDULUM_LIMIT` | Pendulum angle exceeds configured limit | Fail closed |
| `0x00000100` | `STATE_SAFETY_FAULT_ARM_LIMIT` | Arm angle exceeds configured limit | Fail closed |
| `0x00000200` | `STATE_SAFETY_FAULT_PENDULUM_RATE_LIMIT` | Pendulum rate exceeds configured limit | Fail closed |
| `0x00000400` | `STATE_SAFETY_FAULT_ARM_RATE_LIMIT` | Arm rate exceeds configured limit | Fail closed |

## Current observe-only profile

The current application profile is configured, with sample-age timeout disabled in `state_safety` (`max_sample_age_us = 0`) and angle/rate bounds set to `FLT_MAX`.

Therefore argument validity, sensor validity, estimator readiness, finite-state checks, and timestamp consistency remain meaningful, while physical plant-envelope limits are intentionally non-restrictive in this profile.

`motor_output_enabled` remains false and the automatic motor sink remains unbound.
