use rip_control::ControlRegime;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultReason {
    Sensor,
    Estimator,
    ControlSafety,
    Authority,
    Watchdog,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeState {
    Disabled,
    Ready,
    Active(ControlRegime),
    Fault(FaultReason),
}
