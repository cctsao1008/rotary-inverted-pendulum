#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlState {
    pub theta_rad: f32,
    pub theta_dot_rad_s: f32,
    pub phi_rad: f32,
    pub phi_dot_rad_s: f32,
    pub timestamp_us: u32,
}

impl ControlState {
    pub const fn new(
        theta_rad: f32,
        theta_dot_rad_s: f32,
        phi_rad: f32,
        phi_dot_rad_s: f32,
        timestamp_us: u32,
    ) -> Self {
        Self {
            theta_rad,
            theta_dot_rad_s,
            phi_rad,
            phi_dot_rad_s,
            timestamp_us,
        }
    }

    pub fn is_finite(&self) -> bool {
        self.theta_rad.is_finite()
            && self.theta_dot_rad_s.is_finite()
            && self.phi_rad.is_finite()
            && self.phi_dot_rad_s.is_finite()
    }

    pub const fn as_vector(&self) -> [f32; 4] {
        [self.theta_rad, self.theta_dot_rad_s, self.phi_rad, self.phi_dot_rad_s]
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EstimatorInput {
    pub theta_rad: f32,
    pub phi_rad: f32,
    pub timestamp_us: u32,
}

impl EstimatorInput {
    pub const fn new(theta_rad: f32, phi_rad: f32, timestamp_us: u32) -> Self {
        Self {
            theta_rad,
            phi_rad,
            timestamp_us,
        }
    }

    pub fn is_finite(&self) -> bool {
        self.theta_rad.is_finite() && self.phi_rad.is_finite()
    }
}
