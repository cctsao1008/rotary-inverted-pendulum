#![no_std]
#![forbid(unsafe_code)]

/// Rotary-pendulum control regime.
///
/// Operational permission and fault state belong to Supervisor, not this enum.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlRegime {
    SwingUp,
    Capture,
    Balance,
}
