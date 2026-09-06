use crate::ControlState;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SafetyLimits {
    pub max_sample_age_us: Option<u32>,
    pub max_abs_pendulum_angle_rad: Option<f32>,
    pub max_abs_arm_angle_rad: Option<f32>,
    pub max_abs_pendulum_rate_rad_s: Option<f32>,
    pub max_abs_arm_rate_rad_s: Option<f32>,
}

impl SafetyLimits {
    pub const fn observe_only() -> Self {
        Self {
            max_sample_age_us: None,
            max_abs_pendulum_angle_rad: None,
            max_abs_arm_angle_rad: None,
            max_abs_pendulum_rate_rad_s: None,
            max_abs_arm_rate_rad_s: None,
        }
    }

    pub fn is_valid(&self) -> bool {
        valid_limit(self.max_abs_pendulum_angle_rad)
            && valid_limit(self.max_abs_arm_angle_rad)
            && valid_limit(self.max_abs_pendulum_rate_rad_s)
            && valid_limit(self.max_abs_arm_rate_rad_s)
    }
}

#[derive(Debug, Clone, Copy)]
pub struct SafetyInput<'a> {
    pub state: &'a ControlState,
    pub sensor_valid: bool,
    pub estimate_ready: bool,
    pub sensor_timestamp_us: u32,
    pub sample_age_us: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FaultSet(u32);

impl FaultSet {
    pub const NONE: Self = Self(0);
    pub const CONFIG: Self = Self(1 << 1);
    pub const SENSOR_INVALID: Self = Self(1 << 2);
    pub const ESTIMATE_NOT_READY: Self = Self(1 << 3);
    pub const STATE_NONFINITE: Self = Self(1 << 4);
    pub const TIMESTAMP: Self = Self(1 << 5);
    pub const SAMPLE_TIMEOUT: Self = Self(1 << 6);
    pub const PENDULUM_LIMIT: Self = Self(1 << 7);
    pub const ARM_LIMIT: Self = Self(1 << 8);
    pub const PENDULUM_RATE_LIMIT: Self = Self(1 << 9);
    pub const ARM_RATE_LIMIT: Self = Self(1 << 10);

    pub const fn bits(self) -> u32 {
        self.0
    }

    pub const fn is_empty(self) -> bool {
        self.0 == 0
    }

    pub const fn contains(self, other: Self) -> bool {
        (self.0 & other.0) == other.0
    }

    fn insert(&mut self, other: Self) {
        self.0 |= other.0;
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SafetyDecision {
    pub faults: FaultSet,
}

impl SafetyDecision {
    pub const fn control_allowed(self) -> bool {
        self.faults.is_empty()
    }
}

pub fn evaluate(input: SafetyInput<'_>, limits: &SafetyLimits) -> SafetyDecision {
    let mut faults = FaultSet::NONE;

    if !limits.is_valid() {
        faults.insert(FaultSet::CONFIG);
    }

    if !input.sensor_valid {
        faults.insert(FaultSet::SENSOR_INVALID);
    }

    if !input.estimate_ready {
        faults.insert(FaultSet::ESTIMATE_NOT_READY);
    }

    if !input.state.is_finite() {
        faults.insert(FaultSet::STATE_NONFINITE);
    }

    if input.state.timestamp_us != input.sensor_timestamp_us {
        faults.insert(FaultSet::TIMESTAMP);
    }

    if let Some(max_age) = limits.max_sample_age_us {
        if input.sample_age_us > max_age {
            faults.insert(FaultSet::SAMPLE_TIMEOUT);
        }
    }

    check_limit(
        input.state.pendulum_angle_rad,
        limits.max_abs_pendulum_angle_rad,
        FaultSet::PENDULUM_LIMIT,
        &mut faults,
    );
    check_limit(
        input.state.arm_angle_rad,
        limits.max_abs_arm_angle_rad,
        FaultSet::ARM_LIMIT,
        &mut faults,
    );
    check_limit(
        input.state.pendulum_rate_rad_s,
        limits.max_abs_pendulum_rate_rad_s,
        FaultSet::PENDULUM_RATE_LIMIT,
        &mut faults,
    );
    check_limit(
        input.state.arm_rate_rad_s,
        limits.max_abs_arm_rate_rad_s,
        FaultSet::ARM_RATE_LIMIT,
        &mut faults,
    );

    SafetyDecision { faults }
}

fn valid_limit(limit: Option<f32>) -> bool {
    match limit {
        Some(value) => value.is_finite() && value > 0.0,
        None => true,
    }
}

fn check_limit(value: f32, limit: Option<f32>, fault: FaultSet, faults: &mut FaultSet) {
    if let Some(limit) = limit {
        if value.is_finite() && value.abs() > limit {
            faults.insert(fault);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn observe_only_limits_keep_structural_checks_active() {
        let state = ControlState::new(0.1, 0.2, 0.3, 0.4, 100);
        let decision = evaluate(
            SafetyInput {
                state: &state,
                sensor_valid: true,
                estimate_ready: true,
                sensor_timestamp_us: 100,
                sample_age_us: 50_000,
            },
            &SafetyLimits::observe_only(),
        );

        assert!(decision.control_allowed());
    }

    #[test]
    fn invalid_sensor_denies_control() {
        let state = ControlState::new(0.0, 0.0, 0.0, 0.0, 100);
        let decision = evaluate(
            SafetyInput {
                state: &state,
                sensor_valid: false,
                estimate_ready: true,
                sensor_timestamp_us: 100,
                sample_age_us: 0,
            },
            &SafetyLimits::observe_only(),
        );

        assert!(decision.faults.contains(FaultSet::SENSOR_INVALID));
        assert!(!decision.control_allowed());
    }
}
