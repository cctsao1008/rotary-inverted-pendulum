#![no_std]
#![forbid(unsafe_code)]

use libm::{cosf, expf, roundf};
use rip_robot_domain::{
    AngleRad, AngularRateRadPerSec, EstimatedState, GeneralizedDemand, StateValidity, TimestampUs,
    TorqueNm,
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

/// Optional recenter trajectory after the Balance handoff has removed most arm rate.
///
/// `commanded_orientation_rad` is an orientation modulo one revolution. The
/// controller chooses the nearest equivalent unwrapped branch; canonical `phi`
/// itself remains continuous/unwrapped physical history.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BalanceRecenterConfig {
    commanded_orientation_rad: f32,
    start_max_abs_arm_rate_rad_s: f32,
    max_reference_rate_rad_s: f32,
    position_time_constant_s: f32,
}

impl BalanceRecenterConfig {
    pub fn new(
        commanded_orientation_rad: f32,
        start_max_abs_arm_rate_rad_s: f32,
        max_reference_rate_rad_s: f32,
        position_time_constant_s: f32,
    ) -> Option<Self> {
        if !commanded_orientation_rad.is_finite()
            || !positive(start_max_abs_arm_rate_rad_s)
            || !positive(max_reference_rate_rad_s)
            || !positive(position_time_constant_s)
        {
            return None;
        }
        Some(Self {
            commanded_orientation_rad,
            start_max_abs_arm_rate_rad_s,
            max_reference_rate_rad_s,
            position_time_constant_s,
        })
    }

    pub const fn commanded_orientation_rad(self) -> f32 {
        self.commanded_orientation_rad
    }

    pub const fn start_max_abs_arm_rate_rad_s(self) -> f32 {
        self.start_max_abs_arm_rate_rad_s
    }

    pub const fn max_reference_rate_rad_s(self) -> f32 {
        self.max_reference_rate_rad_s
    }

    pub const fn position_time_constant_s(self) -> f32 {
        self.position_time_constant_s
    }
}

/// Optional moving arm reference used only after Capture has earned Balance.
///
/// The reference preserves canonical continuous/unwrapped arm state and instead
/// changes the coordinates seen by the Balance controller. At handoff the
/// current arm angle/rate are latched as zero tracking error; the rate reference
/// first decays toward zero. If recentering is enabled, the stopped arm is then
/// guided to the nearest unwrapped branch equivalent to the commanded periodic
/// orientation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BalanceReferenceConfig {
    arm_rate_decay_time_constant_s: f32,
    recenter: Option<BalanceRecenterConfig>,
}

impl BalanceReferenceConfig {
    pub fn new(arm_rate_decay_time_constant_s: f32) -> Option<Self> {
        positive(arm_rate_decay_time_constant_s).then_some(Self {
            arm_rate_decay_time_constant_s,
            recenter: None,
        })
    }

    pub const fn arm_rate_decay_time_constant_s(self) -> f32 {
        self.arm_rate_decay_time_constant_s
    }

    pub const fn recenter(self) -> Option<BalanceRecenterConfig> {
        self.recenter
    }

    pub const fn with_recenter(mut self, recenter: BalanceRecenterConfig) -> Self {
        self.recenter = Some(recenter);
        self
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BalanceReferencePhase {
    SpinDown,
    Recenter,
    Hold,
}

impl BalanceReferencePhase {
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::SpinDown => "spin_down",
            Self::Recenter => "recenter",
            Self::Hold => "hold",
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BalanceReferenceState {
    pub phi_ref: AngleRad,
    pub phi_dot_ref: AngularRateRadPerSec,
    pub updated_at: TimestampUs,
    pub phase: BalanceReferencePhase,
    pub recenter_target_phi: Option<AngleRad>,
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
    balance_reference_config: Option<BalanceReferenceConfig>,
    balance_reference: Option<BalanceReferenceState>,
}

impl HybridController {
    /// Construct the legacy/global-zero Balance controller.
    ///
    /// Existing firmware/shadow call sites keep their current semantics. A
    /// moving arm reference must be enabled explicitly by the caller.
    pub const fn new(
        swing_up: EnergySwingUpController,
        balance: LqrController,
        capture: CapturePolicy,
    ) -> Self {
        Self {
            swing_up,
            balance,
            capture,
            balance_reference_config: None,
            balance_reference: None,
        }
    }

    pub const fn new_with_balance_reference(
        swing_up: EnergySwingUpController,
        balance: LqrController,
        capture: CapturePolicy,
        balance_reference_config: BalanceReferenceConfig,
    ) -> Self {
        Self {
            swing_up,
            balance,
            capture,
            balance_reference_config: Some(balance_reference_config),
            balance_reference: None,
        }
    }

    pub const fn regime(&self) -> ControlRegime {
        self.capture.regime()
    }

    pub const fn balance_reference(&self) -> Option<BalanceReferenceState> {
        self.balance_reference
    }

    pub fn reset(&mut self) {
        self.capture.reset();
        self.balance_reference = None;
    }

    fn balance_tracking_state(
        &mut self,
        state: &EstimatedState,
        entered_balance: bool,
    ) -> Result<EstimatedState, HybridControlError> {
        let Some(config) = self.balance_reference_config else {
            self.balance_reference = None;
            return Ok(*state);
        };

        let reference = if entered_balance || self.balance_reference.is_none() {
            BalanceReferenceState {
                phi_ref: state.phi,
                phi_dot_ref: state.phi_dot,
                updated_at: state.timestamp,
                phase: BalanceReferencePhase::SpinDown,
                recenter_target_phi: None,
            }
        } else {
            advance_balance_reference(
                self.balance_reference.expect("balance reference checked above"),
                state,
                config,
                self.capture.config(),
            )
            .ok_or(HybridControlError::Numeric)?
        };
        self.balance_reference = Some(reference);

        let tracking_state = EstimatedState {
            phi: AngleRad(state.phi.0 - reference.phi_ref.0),
            phi_dot: AngularRateRadPerSec(state.phi_dot.0 - reference.phi_dot_ref.0),
            ..*state
        };
        if tracking_state.is_finite() {
            Ok(tracking_state)
        } else {
            Err(HybridControlError::Numeric)
        }
    }
}

impl Controller for HybridController {
    type Error = HybridControlError;

    fn compute(&mut self, state: &EstimatedState) -> Result<GeneralizedDemand, Self::Error> {
        let previous_regime = self.capture.regime();
        let regime = self
            .capture
            .update(state)
            .map_err(HybridControlError::Capture)?;

        if regime != ControlRegime::Balance {
            self.balance_reference = None;
        }

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
            ControlRegime::Balance => {
                let tracking_state = self.balance_tracking_state(
                    state,
                    previous_regime != ControlRegime::Balance,
                )?;
                self.balance
                    .compute(&tracking_state)
                    .map_err(HybridControlError::Balance)
            }
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

fn advance_balance_reference(
    reference: BalanceReferenceState,
    state: &EstimatedState,
    config: BalanceReferenceConfig,
    capture_config: CapturePolicyConfig,
) -> Option<BalanceReferenceState> {
    let delta_us = state.timestamp.0.checked_sub(reference.updated_at.0)?;
    let dt_s = delta_us as f32 * 1.0e-6;
    if !dt_s.is_finite() {
        return None;
    }
    if dt_s == 0.0 {
        return Some(BalanceReferenceState {
            updated_at: state.timestamp,
            ..reference
        });
    }

    match reference.phase {
        BalanceReferencePhase::SpinDown => {
            let tau_s = config.arm_rate_decay_time_constant_s();
            let decay = expf(-dt_s / tau_s);
            if !decay.is_finite() || !(0.0..=1.0).contains(&decay) {
                return None;
            }

            let phi_dot_0 = reference.phi_dot_ref.0;
            let phi_dot_ref = phi_dot_0 * decay;
            let phi_ref = reference.phi_ref.0 + phi_dot_0 * tau_s * (1.0 - decay);
            if !phi_ref.is_finite() || !phi_dot_ref.is_finite() {
                return None;
            }

            let mut advanced = BalanceReferenceState {
                phi_ref: AngleRad(phi_ref),
                phi_dot_ref: AngularRateRadPerSec(phi_dot_ref),
                updated_at: state.timestamp,
                phase: BalanceReferencePhase::SpinDown,
                recenter_target_phi: None,
            };

            if let Some(recenter) = config.recenter() {
                let arm_rate_ready = state.phi_dot.0.abs()
                    <= recenter.start_max_abs_arm_rate_rad_s()
                    && phi_dot_ref.abs() <= recenter.start_max_abs_arm_rate_rad_s();
                let pendulum_ready = within(
                    state,
                    capture_config.balance_enter_angle_rad,
                    capture_config.balance_enter_rate_rad_s,
                );
                if arm_rate_ready && pendulum_ready {
                    let target = nearest_equivalent_arm_orientation(
                        state.phi.0,
                        recenter.commanded_orientation_rad(),
                    )?;
                    advanced.phase = BalanceReferencePhase::Recenter;
                    advanced.phi_dot_ref = AngularRateRadPerSec(0.0);
                    advanced.recenter_target_phi = Some(AngleRad(target));
                }
            }

            Some(advanced)
        }
        BalanceReferencePhase::Recenter => {
            let recenter = config.recenter()?;
            let target = reference.recenter_target_phi?.0;
            let error = target - reference.phi_ref.0;
            if !error.is_finite() {
                return None;
            }

            if error.abs() <= 1.0e-4 {
                return Some(BalanceReferenceState {
                    phi_ref: AngleRad(target),
                    phi_dot_ref: AngularRateRadPerSec(0.0),
                    updated_at: state.timestamp,
                    phase: BalanceReferencePhase::Hold,
                    recenter_target_phi: Some(AngleRad(target)),
                });
            }

            let desired_rate = (error / recenter.position_time_constant_s()).clamp(
                -recenter.max_reference_rate_rad_s(),
                recenter.max_reference_rate_rad_s(),
            );
            let step = desired_rate * dt_s;
            if !desired_rate.is_finite() || !step.is_finite() {
                return None;
            }

            if step.abs() >= error.abs() {
                Some(BalanceReferenceState {
                    phi_ref: AngleRad(target),
                    phi_dot_ref: AngularRateRadPerSec(0.0),
                    updated_at: state.timestamp,
                    phase: BalanceReferencePhase::Hold,
                    recenter_target_phi: Some(AngleRad(target)),
                })
            } else {
                Some(BalanceReferenceState {
                    phi_ref: AngleRad(reference.phi_ref.0 + step),
                    phi_dot_ref: AngularRateRadPerSec(desired_rate),
                    updated_at: state.timestamp,
                    phase: BalanceReferencePhase::Recenter,
                    recenter_target_phi: Some(AngleRad(target)),
                })
            }
        }
        BalanceReferencePhase::Hold => Some(BalanceReferenceState {
            phi_dot_ref: AngularRateRadPerSec(0.0),
            updated_at: state.timestamp,
            ..reference
        }),
    }
}

fn nearest_equivalent_arm_orientation(around_phi: f32, commanded_orientation: f32) -> Option<f32> {
    if !around_phi.is_finite() || !commanded_orientation.is_finite() {
        return None;
    }
    let revolution = 2.0 * core::f32::consts::PI;
    let branch = roundf((around_phi - commanded_orientation) / revolution);
    let target = commanded_orientation + branch * revolution;
    target.is_finite().then_some(target)
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

    fn state(theta: f32, theta_dot: f32) -> EstimatedState {
        state_with_arm(theta, theta_dot, 0.0, 0.0)
    }

    fn state_with_arm(theta: f32, theta_dot: f32, phi: f32, phi_dot: f32) -> EstimatedState {
        state_with_arm_at(1_000, theta, theta_dot, phi, phi_dot)
    }

    fn state_with_arm_at(
        timestamp_us: u64,
        theta: f32,
        theta_dot: f32,
        phi: f32,
        phi_dot: f32,
    ) -> EstimatedState {
        EstimatedState {
            timestamp: TimestampUs(timestamp_us),
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
    fn balance_restores_full_arm_state_feedback_when_reference_handoff_is_disabled() {
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
    fn balance_reference_handoff_starts_with_zero_arm_error_and_decays_rate_reference() {
        let gains = [0.2, 0.02, 0.01, 0.01];
        let balance = LqrController::new(gains).unwrap();
        let reference_config = BalanceReferenceConfig::new(1.0).unwrap();
        let mut hybrid = HybridController::new_with_balance_reference(
            swing(),
            balance,
            policy(),
            reference_config,
        );

        hybrid
            .compute(&state_with_arm_at(1_000, 0.30, 0.5, 10.0, -20.0))
            .unwrap();
        hybrid
            .compute(&state_with_arm_at(2_000, 0.10, 0.4, 10.0, -20.0))
            .unwrap();
        hybrid
            .compute(&state_with_arm_at(3_000, 0.10, 0.4, 10.0, -20.0))
            .unwrap();
        let entry = hybrid
            .compute(&state_with_arm_at(4_000, 0.10, 0.4, 10.0, -20.0))
            .unwrap();
        assert_eq!(hybrid.regime(), ControlRegime::Balance);
        assert!((entry.arm_torque.0 - -0.028).abs() < 1.0e-6);

        let latched = hybrid.balance_reference().unwrap();
        assert_eq!(latched.phi_ref, AngleRad(10.0));
        assert_eq!(latched.phi_dot_ref, AngularRateRadPerSec(-20.0));
        assert_eq!(latched.phase, BalanceReferencePhase::SpinDown);

        let _ = hybrid
            .compute(&state_with_arm_at(104_000, 0.10, 0.4, 10.0, -20.0))
            .unwrap();
        let advanced = hybrid.balance_reference().unwrap();
        assert!(advanced.phi_ref.0 < latched.phi_ref.0);
        assert!(advanced.phi_dot_ref.0.abs() < latched.phi_dot_ref.0.abs());
        assert_eq!(advanced.updated_at, TimestampUs(104_000));
        assert_eq!(advanced.phase, BalanceReferencePhase::SpinDown);
    }

    #[test]
    fn nearest_equivalent_orientation_does_not_unwind_accumulated_revolutions() {
        let stopped_phi = -1096.0 * core::f32::consts::PI / 180.0;
        let target = nearest_equivalent_arm_orientation(stopped_phi, 0.0).unwrap();
        let expected = -3.0 * 2.0 * core::f32::consts::PI;
        assert!((target - expected).abs() < 1.0e-5);
        assert!((target - stopped_phi).abs() < core::f32::consts::PI);
    }

    #[test]
    fn recenter_waits_for_spin_down_then_moves_reference_to_nearest_branch() {
        let gains = [0.2, 0.02, 0.01, 0.01];
        let balance = LqrController::new(gains).unwrap();
        let recenter = BalanceRecenterConfig::new(0.0, 0.25, 0.25, 0.5).unwrap();
        let reference_config = BalanceReferenceConfig::new(1.0)
            .unwrap()
            .with_recenter(recenter);
        let mut hybrid = HybridController::new_with_balance_reference(
            swing(),
            balance,
            policy(),
            reference_config,
        );
        let stopped_phi = -1096.0 * core::f32::consts::PI / 180.0;

        hybrid
            .compute(&state_with_arm_at(1_000, 0.30, 0.5, stopped_phi, -0.20))
            .unwrap();
        hybrid
            .compute(&state_with_arm_at(2_000, 0.10, 0.4, stopped_phi, -0.20))
            .unwrap();
        hybrid
            .compute(&state_with_arm_at(3_000, 0.10, 0.4, stopped_phi, -0.20))
            .unwrap();
        hybrid
            .compute(&state_with_arm_at(4_000, 0.10, 0.4, stopped_phi, -0.20))
            .unwrap();
        assert_eq!(hybrid.regime(), ControlRegime::Balance);
        assert_eq!(
            hybrid.balance_reference().unwrap().phase,
            BalanceReferencePhase::SpinDown
        );

        hybrid
            .compute(&state_with_arm_at(1_004_000, 0.05, 0.1, stopped_phi, 0.0))
            .unwrap();
        let recentering = hybrid.balance_reference().unwrap();
        assert_eq!(recentering.phase, BalanceReferencePhase::Recenter);
        let target = recentering.recenter_target_phi.unwrap().0;
        let expected = -3.0 * 2.0 * core::f32::consts::PI;
        assert!((target - expected).abs() < 1.0e-5);

        hybrid
            .compute(&state_with_arm_at(1_104_000, 0.05, 0.1, stopped_phi, 0.0))
            .unwrap();
        let moved = hybrid.balance_reference().unwrap();
        assert_eq!(moved.phase, BalanceReferencePhase::Recenter);
        assert!(moved.phi_ref.0 > recentering.phi_ref.0);
        assert!(moved.phi_dot_ref.0 > 0.0);
    }

    #[test]
    fn balance_reference_is_cleared_when_balance_falls_back_to_capture() {
        let gains = [0.2, 0.02, 0.01, 0.01];
        let balance = LqrController::new(gains).unwrap();
        let mut hybrid = HybridController::new_with_balance_reference(
            swing(),
            balance,
            policy(),
            BalanceReferenceConfig::new(1.0).unwrap(),
        );

        hybrid
            .compute(&state_with_arm_at(1_000, 0.30, 0.5, 3.0, -4.0))
            .unwrap();
        for timestamp in [2_000, 3_000, 4_000] {
            hybrid
                .compute(&state_with_arm_at(timestamp, 0.10, 0.4, 3.0, -4.0))
                .unwrap();
        }
        assert_eq!(hybrid.regime(), ControlRegime::Balance);
        assert!(hybrid.balance_reference().is_some());

        hybrid
            .compute(&state_with_arm_at(5_000, 0.25, 0.5, 3.0, -4.0))
            .unwrap();
        assert_eq!(hybrid.regime(), ControlRegime::Capture);
        assert!(hybrid.balance_reference().is_none());
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
