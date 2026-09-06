#![no_std]
#![forbid(unsafe_code)]

pub mod drive;
pub mod encoder;
pub mod pendulum;

pub use drive::{DriveCommand, DriveDirection, DriveMap, DriveMapError};
pub use encoder::{EncoderScale, EncoderScaleError};
pub use pendulum::{PendulumCalibration, PendulumCalibrationError};

#[cfg(test)]
extern crate std;
