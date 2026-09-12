#![no_std]
#![forbid(unsafe_code)]

use rip_dsp_kernel::dot_f32;
use rip_robot_domain::{EstimatedState, GeneralizedDemand, StateValidity, TorqueNm};

/// Reference-backed QNET RIP LQR gains expressed in the project's canonical
/// state order `[theta, theta_dot, phi, phi_dot]` and arm-torque domain.
///
/// Abdullah et al. (2021) report voltage-domain gains in paper state order
/// `[arm_angle, pendulum_angle, arm_rate, pendulum_rate]` as
/// `[-2.24, 36.71, -1.49, 3.17]` with `V = -Kx`. The paper's positive
/// pendulum-angle direction is opposite the project's positive `theta`
/// direction, so `paper_pendulum_angle = -theta` and
/// `paper_pendulum_rate = -theta_dot`, while the arm coordinates map directly
/// to `phi` and `phi_dot`.
///
/// Applying that coordinate transform first gives the project voltage-domain
/// vector `[-36.71, -3.17, -2.24, -1.49]`. Multiplying by
/// `Kt / Rm = 0.042 / 8.4 = 0.005 N*m/V` gives this torque-feedback vector.
/// It is a nominal reference profile, not Forest D1 specimen calibration.
pub const QNET_REFERENCE_TORQUE_GAINS: [f32; 4] =
    [-0.183_55, -0.015_85, -0.011_20, -0.007_45];

pub trait Controller {
    type Error;

    fn compute(&mut self, state: &EstimatedState) -> Result<GeneralizedDemand, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LqrConfigError {
    NonFiniteGain,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LqrError {
    InvalidState,
    Numeric,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LqrController {
    gains: [f32; 4],
}

impl LqrController {
    pub fn new(gains: [f32; 4]) -> Result<Self, LqrConfigError> {
        if gains.iter().any(|gain| !gain.is_finite()) {
            return Err(LqrConfigError::NonFiniteGain);
        }
        Ok(Self { gains })
    }

    pub const fn gains(self) -> [f32; 4] {
        self.gains
    }
}

impl Controller for LqrController {
    type Error = LqrError;

    fn compute(&mut self, state: &EstimatedState) -> Result<GeneralizedDemand, Self::Error> {
        if state.validity != StateValidity::Valid || !state.is_finite() {
            return Err(LqrError::InvalidState);
        }

        let state_vector = state.as_vector();
        let feedback = dot_f32(&self.gains, &state_vector);
        if !feedback.is_finite() {
            return Err(LqrError::Numeric);
        }

        Ok(GeneralizedDemand {
            arm_torque: TorqueNm(-feedback),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rip_robot_domain::{AngleRad, AngularRateRadPerSec, TimestampUs};

    #[test]
    fn lqr_uses_theta_theta_dot_phi_phi_dot_order() {
        let mut controller = LqrController::new([2.0, 3.0, 5.0, 7.0]).unwrap();
        let state = EstimatedState {
            timestamp: TimestampUs(10),
            theta: AngleRad(1.0),
            theta_dot: AngularRateRadPerSec(2.0),
            phi: AngleRad(3.0),
            phi_dot: AngularRateRadPerSec(4.0),
            validity: StateValidity::Valid,
        };

        assert_eq!(
            controller.compute(&state).unwrap().arm_torque,
            TorqueNm(-51.0)
        );
    }

    #[test]
    fn qnet_reference_profile_has_canonical_project_order_and_signs() {
        assert_eq!(
            QNET_REFERENCE_TORQUE_GAINS,
            [-0.183_55, -0.015_85, -0.011_20, -0.007_45]
        );
    }

    #[test]
    fn qnet_reference_feedback_commands_positive_torque_for_positive_theta() {
        let mut controller = LqrController::new(QNET_REFERENCE_TORQUE_GAINS).unwrap();
        let state = EstimatedState {
            timestamp: TimestampUs(10),
            theta: AngleRad(0.1),
            theta_dot: AngularRateRadPerSec(0.0),
            phi: AngleRad(0.0),
            phi_dot: AngularRateRadPerSec(0.0),
            validity: StateValidity::Valid,
        };

        let demand = controller.compute(&state).unwrap();
        assert!(demand.arm_torque.0 > 0.0);
    }

    #[test]
    fn invalid_state_is_rejected() {
        let mut controller = LqrController::new([1.0, 1.0, 1.0, 1.0]).unwrap();
        let state = EstimatedState::default();
        assert_eq!(controller.compute(&state), Err(LqrError::InvalidState));
    }
}
