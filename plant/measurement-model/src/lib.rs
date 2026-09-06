#![no_std]
#![forbid(unsafe_code)]

use core::f32::consts::PI;
use rip_robot_domain::AngleRad;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PendulumCalibrationError {
    InvalidScale,
    InvalidDirection,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PendulumCalibration {
    upright_adc: i32,
    radians_per_count: f32,
    direction: f32,
}

impl PendulumCalibration {
    pub fn new(
        upright_adc: u16,
        radians_per_count: f32,
        direction: i8,
    ) -> Result<Self, PendulumCalibrationError> {
        if !radians_per_count.is_finite() || radians_per_count <= 0.0 {
            return Err(PendulumCalibrationError::InvalidScale);
        }
        if direction != -1 && direction != 1 {
            return Err(PendulumCalibrationError::InvalidDirection);
        }

        Ok(Self {
            upright_adc: upright_adc as i32,
            radians_per_count,
            direction: direction as f32,
        })
    }

    pub fn angle(self, raw_adc: u16) -> AngleRad {
        AngleRad(wrap_pi(
            (raw_adc as i32 - self.upright_adc) as f32 * self.radians_per_count * self.direction,
        ))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncoderScaleError {
    InvalidCountsPerRevolution,
    InvalidDirection,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EncoderScale {
    radians_per_count: f32,
    direction: f32,
}

impl EncoderScale {
    pub fn new(counts_per_revolution: f32, direction: i8) -> Result<Self, EncoderScaleError> {
        if !counts_per_revolution.is_finite() || counts_per_revolution <= 0.0 {
            return Err(EncoderScaleError::InvalidCountsPerRevolution);
        }
        if direction != -1 && direction != 1 {
            return Err(EncoderScaleError::InvalidDirection);
        }

        Ok(Self {
            radians_per_count: (2.0 * PI) / counts_per_revolution,
            direction: direction as f32,
        })
    }

    pub fn angle(self, accumulated_count: i32) -> AngleRad {
        AngleRad(accumulated_count as f32 * self.radians_per_count * self.direction)
    }
}

fn wrap_pi(mut value: f32) -> f32 {
    let tau = 2.0 * PI;
    while value > PI {
        value -= tau;
    }
    while value < -PI {
        value += tau;
    }
    value
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn encoder_angle_is_continuous_across_revolutions() {
        let scale = EncoderScale::new(1040.0, 1).unwrap();
        let angle = scale.angle(2080).0;
        assert!((angle - 4.0 * PI).abs() < 1.0e-5);
    }

    #[test]
    fn pendulum_angle_is_wrapped_about_upright() {
        let calibration = PendulumCalibration::new(2048, 0.01, 1).unwrap();
        let angle = calibration.angle(2450).0;
        assert!((-PI..=PI).contains(&angle));
    }
}
