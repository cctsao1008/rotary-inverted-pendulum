#![no_std]
#![forbid(unsafe_code)]

pub mod controller;
pub mod effort;
pub mod estimator;
pub mod regime;
pub mod safety;
pub mod state;

pub use controller::{
    lqr::{LqrConfigError, LqrController, LqrError},
    Controller,
};
pub use effort::{ControlEffort, EffortError};
pub use estimator::{BasicEstimator, Estimate, EstimatorConfig, EstimatorError};
pub use regime::ControlRegime;
pub use safety::{ControlSafety, FaultSet, SafetyDecision, SafetyLimits};
pub use state::{ControlState, EstimatorInput};

#[cfg(test)]
extern crate std;
