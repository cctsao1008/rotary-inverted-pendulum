#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlState {
    pub pendulum_angle_rad: f32,
    pub pendulum_rate_rad_s: f32,
    pub arm_angle_rad: f32,
    pub arm_rate_rad_s: f32,
    pub timestamp_us: u32,
}

impl ControlState {
    pub const fn new(
        pendulum_angle_rad: f32,
        pendulum_rate_rad_s: f32,
        arm_angle_rad: f32,
        arm_rate_rad_s: f32,
        timestamp_us: u32,
    ) -> Self {
        Self {
            pendulum_angle_rad,
            pendulum_rate_rad_s,
            arm_angle_rad,
            arm_rate_rad_s,
            timestamp_us,
        }
    }

    pub fn is_finite(&self) -> bool {
        self.pendulum_angle_rad.is_finite()
            && self.pendulum_rate_rad_s.is_finite()
            && self.arm_angle_rad.is_finite()
            && self.arm_rate_rad_s.is_finite()
    }

    pub const fn as_vector(&self) -> [f32; 4] {
        [
            self.pendulum_angle_rad,
            self.pendulum_rate_rad_s,
            self.arm_angle_rad,
            self.arm_rate_rad_s,
        ]
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EstimatorInput {
    pub pendulum_angle_rad: f32,
    pub arm_angle_rad: f32,
    pub timestamp_us: u32,
}

impl EstimatorInput {
    pub const fn new(
        pendulum_angle_rad: f32,
        arm_angle_rad: f32,
        timestamp_us: u32,
    ) -> Self {
        Self {
            pendulum_angle_rad,
            arm_angle_rad,
            timestamp_us,
        }
    }

    pub fn is_finite(&self) -> bool {
        self.pendulum_angle_rad.is_finite() && self.arm_angle_rad.is_finite()
    }
}
