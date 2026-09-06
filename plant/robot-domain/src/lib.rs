#![no_std]
#![forbid(unsafe_code)]

/// Angle in radians.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AngleRad(pub f32);

/// Angular rate in radians per second.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct AngularRateRadPerSec(pub f32);

/// Torque in newton-metres.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TorqueNm(pub f32);

/// Timestamp in microseconds on the firmware monotonic timebase.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct TimestampUs(pub u64);

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum StateValidity {
    #[default]
    Invalid,
    Valid,
}

/// Estimated state presented to the Control domain.
///
/// State-vector order is `[theta, theta_dot, phi, phi_dot]`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EstimatedState {
    pub timestamp: TimestampUs,
    pub theta: AngleRad,
    pub theta_dot: AngularRateRadPerSec,
    pub phi: AngleRad,
    pub phi_dot: AngularRateRadPerSec,
    pub validity: StateValidity,
}

impl EstimatedState {
    pub const fn as_vector(self) -> [f32; 4] {
        [self.theta.0, self.theta_dot.0, self.phi.0, self.phi_dot.0]
    }

    pub fn is_finite(self) -> bool {
        self.theta.0.is_finite()
            && self.theta_dot.0.is_finite()
            && self.phi.0.is_finite()
            && self.phi_dot.0.is_finite()
    }
}

/// Physical generalized input requested by the Control domain.
///
/// The Furuta plant input is rotary-arm torque. This type deliberately carries
/// no PWM, GPIO, direction, or H-bridge semantics.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct GeneralizedDemand {
    pub arm_torque: TorqueNm,
}

/// Bounded actuator request in the abstract actuator domain.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct NormalizedCommand(f32);

impl NormalizedCommand {
    pub const ZERO: Self = Self(0.0);

    pub fn new(value: f32) -> Option<Self> {
        if value.is_finite() && (-1.0..=1.0).contains(&value) {
            Some(Self(value))
        } else {
            None
        }
    }

    pub const fn get(self) -> f32 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn state_vector_order_is_theta_theta_dot_phi_phi_dot() {
        let state = EstimatedState {
            timestamp: TimestampUs(10),
            theta: AngleRad(1.0),
            theta_dot: AngularRateRadPerSec(2.0),
            phi: AngleRad(3.0),
            phi_dot: AngularRateRadPerSec(4.0),
            validity: StateValidity::Valid,
        };

        assert_eq!(state.as_vector(), [1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn normalized_command_rejects_nonfinite_and_out_of_range_values() {
        assert!(NormalizedCommand::new(1.1).is_none());
        assert!(NormalizedCommand::new(f32::NAN).is_none());
        assert_eq!(NormalizedCommand::new(-0.5).unwrap().get(), -0.5);
    }
}
