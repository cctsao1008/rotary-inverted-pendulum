use core::f32::consts::PI;

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

    pub fn angle_rad(&self, accumulated_count: i32) -> f32 {
        accumulated_count as f32 * self.radians_per_count * self.direction
    }
}
