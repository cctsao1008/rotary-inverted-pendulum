#![no_std]
#![forbid(unsafe_code)]

use rip_runtime_state::{AuthorizedActuation, MaintenanceActuation};

/// Firmware-owned physical-output sink.
///
/// Closed-loop output requires an `AuthorizedActuation` proof token. Maintenance
/// output uses a distinct Supervisor-owned proof type.
pub trait ActuationSink {
    type Error;

    fn apply_closed_loop(&mut self, actuation: AuthorizedActuation) -> Result<(), Self::Error>;
    fn apply_maintenance(&mut self, actuation: MaintenanceActuation) -> Result<(), Self::Error>;
    fn safe_off(&mut self) -> Result<(), Self::Error>;
}
