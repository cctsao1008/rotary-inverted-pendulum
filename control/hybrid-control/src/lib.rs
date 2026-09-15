#![no_std]
#![forbid(unsafe_code)]

use libm::cosf;
use rip_robot_domain::{
    AngleRad, AngularRateRadPerSec, EstimatedState, GeneralizedDemand, StateValidity, TorqueNm,
};
use rip_state_feedback::{Controller, LqrController, LqrError};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ControlRegime {
    SwingUp,
    Capture,
    Balance,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnergySwingUpConfig {
    pub pendulum_mass_kg: f32,
    pub pendulum_com_length_m: f32,
    pub pendulum_inertia_kg_m2: f32,
    pub gravity_m_s2: f32,
    pub target_energy_j: f32,
    pub energy_gain: f32,
    pub max_abs_torque_nm: f32,
    pub kick_torque_nm: f32,
    pub kick_below_rate_rad_s: f32,
}

impl EnergySwingUpConfig {
    pub fn is_valid(self) -> bool {
        positive(self.pendulum_mass_kg)
            && positive(self.pendulum_com_length_m)
            && positive(self.pendulum_inertia_kg_m2)
            && positive(self.gravity_m_s2)
            && nonnegative(self.target_energy_j)
            && positive(self.energy_gain)
            && positive(self.max_abs_torque_nm)
            && nonnegative(self.kick_torque_nm)
            && self.kick_torque_nm <= self.max_abs_torque_nm
            && nonnegative(self.kick_below_rate_rad_s)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SwingUpError {
    InvalidConfig,
    InvalidState,
    Numeric,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EnergySwingUpController {
    config: EnergySwingUpConfig,
}

impl EnergySwingUpController {
    pub fn new(config: EnergySwingUpConfig) -> Result<Self, SwingUpError> {
        if !config.is_valid() {
            return Err(SwingUpError::InvalidConfig);
        }
        Ok(Self { config })
    }

    pub const fn config(self) -> EnergySwingUpConfig {
        self.config
    }

    /// Pendulum energy with project theta semantics: theta=0 is upright.
    /// Potential energy is referenced to the hanging-down position.
    pub fn pendulum_energy_j(self, state: &EstimatedState) -> Result<f32, SwingUpError> {
        if state.validity != StateValidity::Valid || !state.is_finite() {
            return Err(SwingUpError::InvalidState);
        }
        let c = cosf(state.theta.0);
        let potential = self.config.pendulum_mass_kg
            * self.config.gravity_m_s2
            * self.config.pendulum_com_length_m
            * (1.0 + c);
        let kinetic =
            0.5 * self.config.pendulum_inertia_kg_m2 * state.theta_dot.0 * state.theta_dot.0;
        let energy = potential + kinetic;
        if energy.is_finite() {
            Ok(energy)
        } else {
            Err(SwingUpError::Numeric)
        }
    }

    pub fn compute(&self, state: &EstimatedState) -> Result<GeneralizedDemand, SwingUpError> {
        let energy = self.pendulum_energy_j(state)?;
        let energy_error = self.config.target_energy_j - energy;

        // Abdullah et al. use u = k (Er-E) alpha_dot cos(alpha), where alpha
        // is measured from hanging-down. For this project theta=0 is upright,
        // so cos(alpha) = -cos(theta).
        let alignment = -cosf(state.theta.0);
        let mut torque = self.config.energy_gain * energy_error * state.theta_dot.0 * alignment;
        if !torque.is_finite() {
            return Err(SwingUpError::Numeric);
        }

        // Energy control has zero authority at an exactly motionless hanging
        // state. A bounded deterministic kick breaks that equilibrium without
        // changing the energy-control law away from the dead-start region.
        if energy_error > 0.0
            && state.theta_dot.0.abs() <= self.config.kick_below_rate_rad_s
            && torque.abs() < self.config.kick_torque_nm
            && self.config.kick_torque_nm > 0.0
        {
            torque = if state.theta.0 < 0.0 {
                -self.config.kick_torque_nm
            } else {
                self.config.kick_torque_nm
            };
        }

        torque = torque.clamp(
            -self.config.max_abs_torque_nm,
            self.config.max_abs_torque_nm,
        );

        Ok(GeneralizedDemand {
            arm_torque: TorqueNm(torque),
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CapturePolicyConfig {
    pub capture_enter_angle_rad: f32,
    // Retained for configuration compatibility and diagnostics. Capture entry
    // is intentionally angle-driven; this value does not gate SwingUp -> Capture.
    pub capture_enter_rate_rad_s: f32,
    pub balance_enter_angle_rad: f32,
    pub balance_enter_rate_rad_s: f32,
    pub balance_exit_angle_rad: f32,
    pub balance_exit_rate_rad_s: f32,
    pub capture_exit_angle_rad: f32,
    // Retained for configuration compatibility and diagnostics. Capture exit
    // uses angle hysteresis; this value does not gate Capture -> SwingUp.
    pub capture_exit_rate_rad_s: f32,
    pub settle_cycles: u16,
}

impl CapturePolicyConfig {
    pub fn is_valid(self) -> bool {
        positive(self.balance_enter_angle_rad)
            && positive(self.capture_enter_angle_rad)
            && positive(self.balance_exit_angle_rad)
            && positive(self.capture_exit_angle_rad)
            && self.balance_enter_angle_rad < self.balance_exit_angle_rad
            && self.balance_exit_angle_rad <= self.capture_enter_angle_rad
            && self.capture_enter_angle_rad < self.capture_exit_angle_rad
            && positive(self.balance_enter_rate_rad_s)
            && positive(self.capture_enter_rate_rad_s)
            && positive(self.balance_exit_rate_rad_s)
            && positive(self.capture_exit_rate_rad_s)
            && self.balance_enter_rate_rad_s < self.balance_exit_rate_rad_s
            && self.balance_exit_rate_rad_s <= self.capture_enter_rate_rad_s
            && self.capture_enter_rate_rad_s < self.capture_exit_rate_rad_s
            && self.settle_cycles > 0
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CapturePolicyError {
    InvalidConfig,
    InvalidState,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CapturePolicy {
    config: CapturePolicyConfig,
    regime: ControlRegime,
    settled_cycles: u16,
}

impl CapturePolicy {
    pub fn new(config: CapturePolicyConfig) -> Result<Self, CapturePolicyError> {
        if !config.is_valid() {
            return Err(CapturePolicyError::InvalidConfig);
        }
        Ok(Self {
            config,
            regime: ControlRegime::SwingUp,
            settled_cycles: 0,
        })
    }

    pub const fn regime(&self) -> ControlRegime {
        self.regime
    }

    pub const fn config(&self) -> CapturePolicyConfig {
        self.config
    }

    pub fn reset(&mut self) {
        self.regime = ControlRegime::SwingUp;
        self.settled_cycles = 0;
    }

    pub fn update(&mut self, state: &EstimatedState) -> Result<ControlRegime, CapturePolicyError> {
        if state.validity != StateValidity::Valid || !state.is_finite() {
            return Err(CapturePolicyError::InvalidState);
        }

        self.regime = match self.regime {
            ControlRegime::SwingUp => {
                // The EBC-to-stabilizer handoff is source-aligned: crossing the
                // capture angle boundary is enough to let the stabilizer catch.
                if within_angle(state, self.config.capture_enter_angle_rad) {
                    self.settled_cycles = 0;
                    ControlRegime::Capture
                } else {
                    ControlRegime::SwingUp
                }
            }
            ControlRegime::Capture => {
                // Capture owns the high-rate catch until angle hysteresis says
                // the attempt failed. Rate is used only for Balance qualification.
                if !within_angle(state, self.config.capture_exit_angle_rad) {
                    self.settled_cycles = 0;
                    ControlRegime::SwingUp
                } else if within(
                    state,
                    self.config.balance_enter_angle_rad,
                    self.config.balance_enter_rate_rad_s,
                ) {
                    self.settled_cycles = self.settled_cycles.saturating_add(1);
                    if self.settled_cycles >= self.config.settle_cycles {
                        self.settled_cycles = 0;
                        ControlRegime::Balance
                    } else {
                        ControlRegime::Capture
                    }
                } else {
                    self.settled_cycles = 0;
                    ControlRegime::Capture
                }
            }
            ControlRegime::Balance => {
                if !within_angle(state, self.config.capture_exit_angle_rad) {
                    self.settled_cycles = 0;
                    ControlRegime::SwingUp
                } else if !within(
                    state,
                    self.config.balance_exit_angle_rad,
                    self.config.balance_exit_rate_rad_s,
                ) {
                    self.settled_cycles = 0;
                    ControlRegime::Capture
                } else {
                    ControlRegime::Balance
                }
            }
        };

        Ok(self.regime)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HybridControlError {
    SwingUp(SwingUpError),
    Capture(CapturePolicyError),
    Balance(LqrError),
    Numeric,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct HybridController {
    swing_up: EnergySwingUpController,
    balance: LqrController,
    capture: CapturePolicy,
}

impl HybridController {
    pub const fn new(
        swing_up: EnergySwingUpController,
        balance: LqrController,
        capture: CapturePolicy,
    ) -> Self {
        Self {
            swing_up,
            balance,
            capture,
        }
    }

    pub const fn regime(&self) -> ControlRegime {
        self.capture.regime()
    }

    pub fn reset(&mut self) {
        self.capture.reset();
    }
}

impl Controller for HybridController {
    type Error = HybridControlError;

    fn compute(&mut self, state: &EstimatedState) -> Result<GeneralizedDemand, Self::Error> {
        let regime = self
            .capture
            .update(state)
            .map_err(HybridControlError::Capture)?;
        match regime {
            ControlRegime::SwingUp => self
                .swing_up
                .compute(state)
                .map_err(HybridControlError::SwingUp),
            ControlRegime::Capture => {
                // Capture is deliberately pendulum-priority. The same feedback
                // gains are reused, but arm-position and arm-rate coordinates are
                // projected out until the pendulum is slow enough to qualify for
                // Balance. This prevents EBC-created arm state from overriding
                // the theta/theta_dot catch demand.
                let projected = pendulum_capture_projection(state);
                self.balance
                    .compute(&projected)
                    .map_err(HybridControlError::Balance)
            }
            ControlRegime::Balance => self
                .balance
                .compute(state)
                .map_err(HybridControlError::Balance),
        }
    }
}

fn pendulum_capture_projection(state: &EstimatedState) -> EstimatedState {
    EstimatedState {
        phi: AngleRad(0.0),
        phi_dot: AngularRateRadPerSec(0.0),
        ..*state
    }
}

fn within_angle(state: &EstimatedState, angle_rad: f32) -> bool {
    state.theta.0.abs() <= angle_rad
}

fn within(state: &EstimatedState, angle_rad: f32, rate_rad_s: f32) -> bool {
    within_angle(state, angle_rad) && state.theta_dot.0.abs() <= rate_rad_s
}

fn positive(value: f32) -> bool {
    value.is_finite() && value > 0.0
}

fn nonnegative(value: f32) -> bool {
    value.is_finite() && value >= 0.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use rip_robot_domain::TimestampUs;

    fn state(theta: f32, theta_dot: f32) -> EstimatedState {
        state_with_arm(theta, theta_dot, 0.0, 0.0)
    }

    fn state_with_arm(theta: f32, theta_dot: f32, phi: f32, phi_dot: f32) -> EstimatedState {
        EstimatedState {
            timestamp: TimestampUs(1_000),
            theta: AngleRad(theta),
            theta_dot: AngularRateRadPerSec(theta_dot),
            phi: AngleRad(phi),
            phi_dot: AngularRateRadPerSec(phi_dot),
            validity: StateValidity::Valid,
        }
    }

    fn swing() -> EnergySwingUpController {
        EnergySwingUpController::new(EnergySwingUpConfig {
            pendulum_mass_kg: 0.04,
            pendulum_com_length_m: 0.129,
            pendulum_inertia_kg_m2: 0.0001,
            gravity_m_s2: 9.81,
            target_energy_j: 0.025,
            energy_gain: 0.175,
            max_abs_torque_nm: 0.05,
            kick_torque_nm: 0.01,
            kick_below_rate_rad_s: 0.05,
        })
        .unwrap()
    }

    fn policy() -> CapturePolicy {
        CapturePolicy::new(CapturePolicyConfig {
            capture_enter_angle_rad: 0.35,
            capture_enter_rate_rad_s: 3.0,
            balance_enter_angle_rad: 0.12,
            balance_enter_rate_rad_s: 1.0,
            balance_exit_angle_rad: 0.20,
            balance_exit_rate_rad_s: 2.0,
            capture_exit_angle_rad: 0.52,
            capture_exit_rate_rad_s: 4.0,
            settle_cycles: 3,
        })
        .unwrap()
    }

    #[test]
    fn energy_is_higher_upright_than_hanging_down() {
        let controller = swing();
        let upright = controller.pendulum_energy_j(&state(0.0, 0.0)).unwrap();
        let down = controller
            .pendulum_energy_j(&state(core::f32::consts::PI, 0.0))
            .unwrap();
        assert!(upright > down);
    }

    #[test]
    fn dead_start_receives_bounded_kick() {
        let demand = swing().compute(&state(core::f32::consts::PI, 0.0)).unwrap();
        assert_eq!(demand.arm_torque, TorqueNm(0.01));
    }

    #[test]
    fn capture_entry_is_angle_only_even_at_high_rate() {
        let mut policy = policy();
        assert_eq!(
            policy.update(&state(0.30, 20.0)).unwrap(),
            ControlRegime::Capture
        );
    }

    #[test]
    fn capture_uses_angle_hysteresis_while_rate_remains_high() {
        let mut policy = policy();
        assert_eq!(
            policy.update(&state(0.30, 20.0)).unwrap(),
            ControlRegime::Capture
        );
        assert_eq!(
            policy.update(&state(0.40, 20.0)).unwrap(),
            ControlRegime::Capture
        );
        assert_eq!(
            policy.update(&state(0.60, 20.0)).unwrap(),
            ControlRegime::SwingUp
        );
    }

    #[test]
    fn capture_requires_settled_cycles_before_balance() {
        let mut policy = policy();
        assert_eq!(
            policy.update(&state(0.30, 0.5)).unwrap(),
            ControlRegime::Capture
        );
        assert_eq!(
            policy.update(&state(0.10, 0.4)).unwrap(),
            ControlRegime::Capture
        );
        assert_eq!(
            policy.update(&state(0.10, 0.4)).unwrap(),
            ControlRegime::Capture
        );
        assert_eq!(
            policy.update(&state(0.10, 0.4)).unwrap(),
            ControlRegime::Balance
        );
    }

    #[test]
    fn balance_falls_back_to_capture_before_swing_up() {
        let mut policy = policy();
        policy.update(&state(0.30, 0.5)).unwrap();
        for _ in 0..3 {
            policy.update(&state(0.10, 0.4)).unwrap();
        }
        assert_eq!(policy.regime(), ControlRegime::Balance);
        assert_eq!(
            policy.update(&state(0.25, 0.5)).unwrap(),
            ControlRegime::Capture
        );
        assert_eq!(
            policy.update(&state(0.60, 0.5)).unwrap(),
            ControlRegime::SwingUp
        );
    }

    #[test]
    fn capture_projects_out_arm_state_before_state_feedback() {
        let gains = [0.2, 0.02, 0.01, 0.01];
        let capture_state = state_with_arm(0.30, 20.0, 5.0, -7.0);
        let projected_state = state(0.30, 20.0);
        let mut expected_controller = LqrController::new(gains).unwrap();
        let expected = expected_controller.compute(&projected_state).unwrap();
        let balance = LqrController::new(gains).unwrap();
        let mut hybrid = HybridController::new(swing(), balance, policy());

        let actual = hybrid.compute(&capture_state).unwrap();
        assert_eq!(hybrid.regime(), ControlRegime::Capture);
        assert_eq!(actual, expected);
    }

    #[test]
    fn balance_restores_full_arm_state_feedback() {
        let gains = [0.2, 0.02, 0.01, 0.01];
        let balance = LqrController::new(gains).unwrap();
        let mut hybrid = HybridController::new(swing(), balance, policy());

        hybrid
            .compute(&state_with_arm(0.30, 0.5, 5.0, -7.0))
            .unwrap();
        for _ in 0..3 {
            hybrid
                .compute(&state_with_arm(0.10, 0.4, 5.0, -7.0))
                .unwrap();
        }
        assert_eq!(hybrid.regime(), ControlRegime::Balance);

        let balance_state = state_with_arm(0.05, 0.2, 1.0, -2.0);
        let mut expected_controller = LqrController::new(gains).unwrap();
        let expected = expected_controller.compute(&balance_state).unwrap();
        let actual = hybrid.compute(&balance_state).unwrap();
        assert_eq!(hybrid.regime(), ControlRegime::Balance);
        assert_eq!(actual, expected);
    }

    #[test]
    fn hybrid_controller_computes_all_three_regimes() {
        let balance = LqrController::new([0.2, 0.02, 0.01, 0.01]).unwrap();
        let mut hybrid = HybridController::new(swing(), balance, policy());

        let first = hybrid.compute(&state(1.0, 0.8)).unwrap();
        assert!(first.arm_torque.0.is_finite());
        assert_eq!(hybrid.regime(), ControlRegime::SwingUp);

        let capture = hybrid.compute(&state(0.30, 0.5)).unwrap();
        assert!(capture.arm_torque.0.is_finite());
        assert_eq!(hybrid.regime(), ControlRegime::Capture);

        for _ in 0..3 {
            hybrid.compute(&state(0.10, 0.4)).unwrap();
        }
        assert_eq!(hybrid.regime(), ControlRegime::Balance);
    }
}
