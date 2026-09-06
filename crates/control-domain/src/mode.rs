#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlMode {
    Disabled,
    Idle,
    SwingUp,
    Capture,
    Balance,
    Fault,
}
