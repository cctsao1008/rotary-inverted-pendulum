use core::f32::consts::PI;

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

    pub fn angle_rad(&self, raw_adc: u16) -> f32 {
        wrap_pi(
            (raw_adc as i32 - self.upright_adc) as f32
                * self.radians_per_count
                * self.direction,
        )
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
