#![no_std]
#![forbid(unsafe_code)]

pub mod actuator;
pub mod controller;
pub mod estimator;
pub mod mode;
pub mod safety;
pub mod state;

pub use actuator::{ActuatorMapper, MotorCommand, MotorDirection, NormalizedEffort};
pub use controller::lqr::{LqrConfigError, LqrController, LqrError};
pub use estimator::{BasicEstimator, Estimate, EstimatorConfig, EstimatorError};
pub use mode::ControlMode;
pub use safety::{FaultSet, SafetyDecision, SafetyInput, SafetyLimits};
pub use state::{ControlState, EstimatorInput};

#[cfg(test)]
extern crate std;
