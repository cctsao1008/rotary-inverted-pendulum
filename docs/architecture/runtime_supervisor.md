# Runtime Supervisor

## Operational state

Operational state is separate from the hybrid-control regime.

```rust
RuntimeState::Disabled
RuntimeState::Ready
RuntimeState::Active(ControlRegime)
RuntimeState::Fault(FaultReason)
```

`ControlRegime` contains only physical control regimes:

```rust
ControlRegime::SwingUp
ControlRegime::Capture
ControlRegime::Balance
```

This separation prevents operational permissions and controller selection from being represented by one overloaded mode enumeration.

## Observe runtime

`ObserveRuntime` implements the non-actuating computation path:

```text
SensorSource
    ↓
Observation
    ↓
BasicEstimator
    ↓
ControlSafety
    ↓
Controller
    ↓
ControlEffort
```

The observe runtime has no `MotorSink` dependency and therefore cannot actuate the plant.

## Motor authority

`MotorAuthority<M>` owns the physical motor sink and keeps it private.

Authority state is runtime-dynamic:

```text
Disarmed
Maintenance
Control
Fault
```

Command access is capability-based:

```text
MotorAuthority
    ├── maintenance_access() -> MaintenanceAccess
    └── control_access()     -> ControlAccess
```

Only those access capabilities expose `apply(DriveCommand)`. A fault or release path forces `safe_off()` before changing ownership state.

This design combines dynamic operational transitions with compiler-enforced restriction of the physical output surface.
