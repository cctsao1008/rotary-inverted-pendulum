#![no_std]
#![forbid(unsafe_code)]

pub mod authority;
pub mod ports;
pub mod runtime;
pub mod state;

pub use authority::{
    AuthorityError, AuthorityState, ControlAccess, MaintenanceAccess, MotorAuthority,
};
pub use ports::{MotorSink, Observation, SensorSource, TelemetrySink};
pub use runtime::{CycleError, ObserveCycle, ObserveRuntime};
pub use state::{FaultReason, RuntimeState};

#[cfg(test)]
extern crate std;
