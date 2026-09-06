use crate::ControlState;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaultSet(u16);

impl FaultSet {
    pub const NONE: Self = Self(0);
    pub const LIMIT_CONFIG: Self = Self(1 << 0);
    pub const SENSOR_INVALID: Self = Self(1 << 1);
    pub const ESTIMATE_NOT_READY: Self = Self(1 << 2);
    pub const STATE_NONFINITE: Self = Self(1 << 3);
    pub const SAMPLE_TIMEOUT: Self = Self(1 << 4);
    pub const THETA_LIMIT: Self = Self(1 << 5);
    pub const PHI_LIMIT: Self = Self(1 << 6);
    pub const THETA_RATE_LIMIT: Self = Self(1 << 7);
    pub const PHI_RATE_LIMIT: Self = Self(1 << 8);

    pub const fn bits(self) -> u16 {
        self.0
    }

    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    const fn with(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SafetyLimits {
    pub max_sample_age_us: Option<u32>,
    pub max_abs_theta_rad: Option<f32>,
    pub max_abs_phi_rad: Option<f32>,
    pub max_abs_theta_dot_rad_s: Option<f32>,
    pub max_abs_phi_dot_rad_s: Option<f32>,
}

impl SafetyLimits {
    pub const fn observe_only() -> Self {
        Self {
            max_sample_age_us: None,
            max_abs_theta_rad: None,
            max_abs_phi_rad: None,
            max_abs_theta_dot_rad_s: None,
            max_abs_phi_dot_rad_s: None,
        }
    }

    pub fn is_valid(&self) -> bool {
        valid_limit(self.max_abs_theta_rad)
            && valid_limit(self.max_abs_phi_rad)
            && valid_limit(self.max_abs_theta_dot_rad_s)
            && valid_limit(self.max_abs_phi_dot_rad_s)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SafetyDecision {
    pub allowed: bool,
    pub faults: FaultSet,
}

pub struct ControlSafety;

impl ControlSafety {
    pub fn check(
        state: Option<ControlState>,
        sensor_valid: bool,
        estimate_ready: bool,
        sample_age_us: u32,
        limits: &SafetyLimits,
    ) -> SafetyDecision {
        let mut faults = FaultSet::NONE;

        if !limits.is_valid() {
            faults = faults.with(FaultSet::LIMIT_CONFIG);
        }
        if !sensor_valid {
            faults = faults.with(FaultSet::SENSOR_INVALID);
        }
        if !estimate_ready || state.is_none() {
            faults = faults.with(FaultSet::ESTIMATE_NOT_READY);
        }
        if let Some(max_age) = limits.max_sample_age_us {
            if sample_age_us > max_age {
                faults = faults.with(FaultSet::SAMPLE_TIMEOUT);
            }
        }

        if let Some(state) = state {
            if !state.is_finite() {
                faults = faults.with(FaultSet::STATE_NONFINITE);
            } else {
                if exceeds(state.theta_rad, limits.max_abs_theta_rad) {
                    faults = faults.with(FaultSet::THETA_LIMIT);
                }
                if exceeds(state.phi_rad, limits.max_abs_phi_rad) {
                    faults = faults.with(FaultSet::PHI_LIMIT);
                }
                if exceeds(state.theta_dot_rad_s, limits.max_abs_theta_dot_rad_s) {
                    faults = faults.with(FaultSet::THETA_RATE_LIMIT);
                }
                if exceeds(state.phi_dot_rad_s, limits.max_abs_phi_dot_rad_s) {
                    faults = faults.with(FaultSet::PHI_RATE_LIMIT);
                }
            }
        }

        SafetyDecision {
            allowed: faults.is_empty(),
            faults,
        }
    }
}

fn valid_limit(limit: Option<f32>) -> bool {
    match limit {
        Some(value) => value.is_finite() && value > 0.0,
        None => true,
    }
}

fn exceeds(value: f32, limit: Option<f32>) -> bool {
    limit.map(|bound| value.abs() > bound).unwrap_or(false)
}
