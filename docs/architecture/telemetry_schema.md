# Telemetry Schema

## Coordinate naming

Mathematical/control documentation uses:

- `phi` (φ): rotary-arm angle
- `theta` (θ): pendulum angle

Software-facing interfaces should use semantic names where available:

- `arm_angle_*`
- `arm_rate_*`
- `pendulum_angle_*`
- `pendulum_rate_*`

Existing human-readable `theta_*` / `phi_*` runtime fields retain their current meaning.

## Current human-readable channels

- `BOOT`: boot and firmware identity
- `BUILD`: build/toolchain identity
- `CONTROL_GATE`: admission/gate state
- `MOTOR_AUTH`: motor-authority state
- `CTRL`: control state, faults, angles, sample age, and gate information
- `PERF`: execution timing/workload metrics

## Structured telemetry status

A versioned machine-readable control record is **not currently implemented**.

Human-readable runtime telemetry is the current implemented observability interface.
