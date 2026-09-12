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
pub const QNET_REFERENCE_TORQUE_GAINS: [f32; 4] = [-0.183_55, -0.015_85, -0.011_20, -0.007_45];

/// Pole-placement C1 baseline derived on the project's QNET nominal upright
/// linearization, using desired poles `{-1, -5, -1-3j, -1+3j}` from Fahmizal
/// (2023). Only the desired pole locations are borrowed from that paper; its
/// plant matrices and controller gains are not copied.
pub const QNET_POLE_PLACEMENT_C1_TORQUE_GAINS: [f32; 4] = [
    -0.036_116_928,
    -0.000_687_032_5,
    -0.000_032_856_984,
    -0.000_255_999_78,
];

/// Pole-placement C2 baseline derived on the project's QNET nominal upright
/// linearization, using desired poles `{-5, -4.1, -5-3j, -5+3j}` from Fahmizal
/// (2023). Only the desired pole locations are borrowed from that paper; its
/// plant matrices and controller gains are not copied.
pub const QNET_POLE_PLACEMENT_C2_TORQUE_GAINS: [f32; 4] = [
    -0.045_846_36,
    -0.002_038_660_6,
    -0.000_458_026_36,
    -0.000_548_032_64,
];

pub trait Controller {
    type Error;

    fn compute(&mut self, state: &EstimatedState) -> Result<GeneralizedDemand, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StateFeedbackConfigError {
    NonFiniteGain,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StateFeedbackError {
    InvalidState,
    Numeric,
}

/// Full-state linear feedback in the project convention `u = -Kx`.
///
/// The implementation deliberately does not encode how `K` was designed.
/// LQR and pole placement are controller-design methods; the runtime state
/// feedback law and its physical torque semantics are identical.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct StateFeedbackController {
    gains: [f32; 4],
}

impl StateFeedbackController {
    pub fn new(gains: [f32; 4]) -> Result<Self, StateFeedbackConfigError> {
        if gains.iter().any(|gain| !gain.is_finite()) {
            return Err(StateFeedbackConfigError::NonFiniteGain);
        }
        Ok(Self { gains })
    }

    pub const fn gains(self) -> [f32; 4] {
        self.gains
    }
}

impl Controller for StateFeedbackController {
    type Error = StateFeedbackError;

    fn compute(&mut self, state: &EstimatedState) -> Result<GeneralizedDemand, Self::Error> {
        if state.validity != StateValidity::Valid || !state.is_finite() {
            return Err(StateFeedbackError::InvalidState);
        }

        let state_vector = state.as_vector();
        let feedback = dot_f32(&self.gains, &state_vector);
        if !feedback.is_finite() {
            return Err(StateFeedbackError::Numeric);
        }

        Ok(GeneralizedDemand {
            arm_torque: TorqueNm(-feedback),
        })
    }
}

/// Compatibility names retained for existing LQR call sites. The runtime law
/// is generic full-state feedback; the design provenance lives in the gain
/// profile selected by the caller.
pub type LqrController = StateFeedbackController;
pub type LqrConfigError = StateFeedbackConfigError;
pub type LqrError = StateFeedbackError;

#[cfg(test)]
mod tests {
    use super::*;
    use rip_robot_domain::{AngleRad, AngularRateRadPerSec, TimestampUs};

    #[test]
    fn state_feedback_uses_theta_theta_dot_phi_phi_dot_order() {
        let mut controller = StateFeedbackController::new([2.0, 3.0, 5.0, 7.0]).unwrap();
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
    fn all_nominal_baselines_command_restorative_torque_for_positive_theta() {
        let state = EstimatedState {
            timestamp: TimestampUs(10),
            theta: AngleRad(0.1),
            theta_dot: AngularRateRadPerSec(0.0),
            phi: AngleRad(0.0),
            phi_dot: AngularRateRadPerSec(0.0),
            validity: StateValidity::Valid,
        };

        for gains in [
            QNET_REFERENCE_TORQUE_GAINS,
            QNET_POLE_PLACEMENT_C1_TORQUE_GAINS,
            QNET_POLE_PLACEMENT_C2_TORQUE_GAINS,
        ] {
            let mut controller = StateFeedbackController::new(gains).unwrap();
            let demand = controller.compute(&state).unwrap();
            assert!(demand.arm_torque.0 > 0.0);
        }
    }

    #[test]
    fn invalid_state_is_rejected() {
        let mut controller = StateFeedbackController::new([1.0, 1.0, 1.0, 1.0]).unwrap();
        let state = EstimatedState::default();
        assert_eq!(
            controller.compute(&state),
            Err(StateFeedbackError::InvalidState)
        );
    }
}
