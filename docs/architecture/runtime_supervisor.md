# Runtime Supervisor

## Operating state and control regime

Operational state is separate from hybrid-control regime.

```rust
RuntimeState::Disabled
RuntimeState::Ready
RuntimeState::Active(ControlRegime)
RuntimeState::Fault(FaultReason)
```

```rust
ControlRegime::SwingUp
ControlRegime::Capture
ControlRegime::Balance
```

## State estimation

State estimation belongs to Supervisor.

```text
EstimatorMeasurement
    ↓
BasicEstimator
    ↓
EstimatedState
```

The estimator uses the shortest circular delta for pendulum angle and continuous delta for rotary-arm angle. Timestamp ordering and maximum-gap handling are estimator runtime validity concerns.

## Runtime qualification

`RuntimePolicy` owns operational qualification of an estimate using:

- sensor-valid evidence;
- sample age;
- estimate readiness/finiteness;
- configured state/rate operating limits.

Physical state definitions remain Plant-owned; the decision whether a sample may participate in runtime control remains Supervisor-owned.

## Timing and watchdog health

`SensorTimingMonitor` classifies the primary observation cadence as `Startup`, `Healthy`, `Late`, or `Timeout`.

`ControlWatchdog` independently classifies control liveness as `Disarmed`, `Healthy`, or `Expired`.

Both are explicit authority evidence.

## Closed-loop authority

Closed-loop authority is a semantic promotion:

```text
BoundedActuatorCommand
        ↓
RuntimeAuthority::evaluate
        ↓
AuthorizedActuation
```

Authorization requires:

- closed-loop authority mode;
- `RuntimeState::Active(...)`;
- healthy sensor timing;
- healthy watchdog;
- valid estimate;
- successful runtime qualification.

Actuator-model saturation remains explicit in the authority decision as a constrained condition.

`AuthorizedActuation` has no public constructor.

## Maintenance authority

Maintenance output uses a separate `MaintenanceActuation` proof type. `RuntimeAuthority` keeps maintenance and closed-loop authority mutually exclusive. Maintenance cannot construct a closed-loop `AuthorizedActuation`.

## Portable control runtime

`ControlRuntime` composes the deterministic portable path:

```text
RuntimeObservation
    ↓
EstimatorMeasurement
    ↓
BasicEstimator
    ↓
RuntimePolicy
    ↓
Controller
    ↓
GeneralizedDemand
    ↓
ArmActuatorModel
    ↓
BoundedActuatorCommand
    ↓
RuntimeAuthority
```

`ControlRuntime` has no physical `ActuationSink` dependency. Firmware remains the only domain capable of realizing electrical output.
