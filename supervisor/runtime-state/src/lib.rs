#![no_std]
#![forbid(unsafe_code)]

use rip_actuator_model::BoundedActuatorCommand;
use rip_hybrid_control::ControlRegime;
use rip_robot_domain::{EstimatedState, StateValidity};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultReason {
    Sensor,
    Estimator,
    RuntimeQualification,
    Authority,
    Watchdog,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RuntimeState {
    Disabled,
    Ready,
    Active(ControlRegime),
    Fault(FaultReason),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct QualificationReasons(u16);

impl QualificationReasons {
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

    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    const fn with(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RuntimeLimits {
    pub max_sample_age_us: Option<u64>,
    pub max_abs_theta_rad: Option<f32>,
    pub max_abs_phi_rad: Option<f32>,
    pub max_abs_theta_dot_rad_s: Option<f32>,
    pub max_abs_phi_dot_rad_s: Option<f32>,
}

impl RuntimeLimits {
    pub const fn observe_only() -> Self {
        Self {
            max_sample_age_us: None,
            max_abs_theta_rad: None,
            max_abs_phi_rad: None,
            max_abs_theta_dot_rad_s: None,
            max_abs_phi_dot_rad_s: None,
        }
    }

    pub fn is_valid(self) -> bool {
        valid_limit(self.max_abs_theta_rad)
            && valid_limit(self.max_abs_phi_rad)
            && valid_limit(self.max_abs_theta_dot_rad_s)
            && valid_limit(self.max_abs_phi_dot_rad_s)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RuntimeQualification {
    pub allowed: bool,
    pub reasons: QualificationReasons,
}

pub struct RuntimePolicy;

impl RuntimePolicy {
    pub fn qualify(
        state: Option<EstimatedState>,
        sensor_valid: bool,
        sample_age_us: u64,
        limits: RuntimeLimits,
    ) -> RuntimeQualification {
        let mut reasons = QualificationReasons::NONE;

        if !limits.is_valid() {
            reasons = reasons.with(QualificationReasons::LIMIT_CONFIG);
        }
        if !sensor_valid {
            reasons = reasons.with(QualificationReasons::SENSOR_INVALID);
        }
        if let Some(max_age) = limits.max_sample_age_us {
            if sample_age_us > max_age {
                reasons = reasons.with(QualificationReasons::SAMPLE_TIMEOUT);
            }
        }

        match state {
            Some(state) if state.validity == StateValidity::Valid => {
                if !state.is_finite() {
                    reasons = reasons.with(QualificationReasons::STATE_NONFINITE);
                } else {
                    if exceeds(state.theta.0, limits.max_abs_theta_rad) {
                        reasons = reasons.with(QualificationReasons::THETA_LIMIT);
                    }
                    if exceeds(state.phi.0, limits.max_abs_phi_rad) {
                        reasons = reasons.with(QualificationReasons::PHI_LIMIT);
                    }
                    if exceeds(state.theta_dot.0, limits.max_abs_theta_dot_rad_s) {
                        reasons = reasons.with(QualificationReasons::THETA_RATE_LIMIT);
                    }
                    if exceeds(state.phi_dot.0, limits.max_abs_phi_dot_rad_s) {
                        reasons = reasons.with(QualificationReasons::PHI_RATE_LIMIT);
                    }
                }
            }
            _ => reasons = reasons.with(QualificationReasons::ESTIMATE_NOT_READY),
        }

        RuntimeQualification {
            allowed: reasons.is_empty(),
            reasons,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum SensorTimingHealth {
    #[default]
    Startup,
    Healthy,
    Late,
    Timeout,
}

impl SensorTimingHealth {
    pub const fn closed_loop_eligible(self) -> bool {
        matches!(self, Self::Healthy)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorTimingLimits {
    expected_period_us: u64,
    late_after_us: u64,
    timeout_after_us: u64,
}

impl SensorTimingLimits {
    pub const fn new(
        expected_period_us: u64,
        late_after_us: u64,
        timeout_after_us: u64,
    ) -> Option<Self> {
        if expected_period_us == 0
            || late_after_us < expected_period_us
            || timeout_after_us <= late_after_us
        {
            None
        } else {
            Some(Self {
                expected_period_us,
                late_after_us,
                timeout_after_us,
            })
        }
    }

    pub const fn classify_elapsed_us(self, elapsed_us: u64) -> SensorTimingHealth {
        if elapsed_us >= self.timeout_after_us {
            SensorTimingHealth::Timeout
        } else if elapsed_us >= self.late_after_us {
            SensorTimingHealth::Late
        } else {
            SensorTimingHealth::Healthy
        }
    }

    pub const fn expected_period_us(self) -> u64 {
        self.expected_period_us
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorTimingMonitor {
    limits: SensorTimingLimits,
    started_at_us: u64,
    last_event_at_us: Option<u64>,
    cadence_verified: bool,
    health: SensorTimingHealth,
}

impl SensorTimingMonitor {
    pub const fn new(limits: SensorTimingLimits, started_at_us: u64) -> Self {
        Self {
            limits,
            started_at_us,
            last_event_at_us: None,
            cadence_verified: false,
            health: SensorTimingHealth::Startup,
        }
    }

    pub fn on_event(&mut self, event_at_us: u64) -> SensorTimingHealth {
        self.health = match self.last_event_at_us {
            Some(previous) => {
                self.cadence_verified = true;
                self.limits
                    .classify_elapsed_us(event_at_us.saturating_sub(previous))
            }
            None => SensorTimingHealth::Startup,
        };
        self.last_event_at_us = Some(event_at_us);
        self.health
    }

    pub fn poll(&mut self, now_us: u64) -> SensorTimingHealth {
        let reference = self.last_event_at_us.unwrap_or(self.started_at_us);
        let elapsed = now_us.saturating_sub(reference);

        self.health = if !self.cadence_verified && elapsed < self.limits.timeout_after_us {
            SensorTimingHealth::Startup
        } else {
            self.limits.classify_elapsed_us(elapsed)
        };
        self.health
    }

    pub const fn health(self) -> SensorTimingHealth {
        self.health
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum WatchdogHealth {
    #[default]
    Disarmed,
    Healthy,
    Expired,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ControlWatchdog {
    timeout_us: u64,
    last_kick_us: Option<u64>,
}

impl ControlWatchdog {
    pub const fn new(timeout_us: u64) -> Option<Self> {
        if timeout_us == 0 {
            None
        } else {
            Some(Self {
                timeout_us,
                last_kick_us: None,
            })
        }
    }

    pub fn kick(&mut self, now_us: u64) {
        self.last_kick_us = Some(now_us);
    }

    pub fn disarm(&mut self) {
        self.last_kick_us = None;
    }

    pub fn health(self, now_us: u64) -> WatchdogHealth {
        match self.last_kick_us {
            None => WatchdogHealth::Disarmed,
            Some(last) if now_us.saturating_sub(last) <= self.timeout_us => WatchdogHealth::Healthy,
            Some(_) => WatchdogHealth::Expired,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AuthorityMode {
    #[default]
    Disarmed,
    ClosedLoop,
    Maintenance,
    Fault,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ActuationAuthority {
    Denied,
    ClosedLoop,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct AuthorityReasons(u16);

impl AuthorityReasons {
    pub const NONE: Self = Self(0);
    pub const MODE: Self = Self(1 << 0);
    pub const RUNTIME_STATE: Self = Self(1 << 1);
    pub const SENSOR_TIMING: Self = Self(1 << 2);
    pub const WATCHDOG: Self = Self(1 << 3);
    pub const ESTIMATE_INVALID: Self = Self(1 << 4);
    pub const RUNTIME_QUALIFICATION: Self = Self(1 << 5);
    pub const ACTUATOR_SATURATED: Self = Self(1 << 6);

    pub const fn bits(self) -> u16 {
        self.0
    }

    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    const fn with(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AuthorityContext {
    pub runtime_state: RuntimeState,
    pub timing: SensorTimingHealth,
    pub watchdog: WatchdogHealth,
    pub estimate_validity: StateValidity,
    pub runtime_qualified: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AuthorityDecision {
    pub authority: ActuationAuthority,
    pub reasons: AuthorityReasons,
    pub constrained: bool,
}

impl AuthorityDecision {
    pub const fn closed_loop_authorized(self) -> bool {
        matches!(self.authority, ActuationAuthority::ClosedLoop)
    }
}

/// Proof that one bounded command passed closed-loop runtime authority.
///
/// There is intentionally no public constructor.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AuthorizedActuation {
    command: BoundedActuatorCommand,
}

impl AuthorizedActuation {
    pub const fn command(self) -> BoundedActuatorCommand {
        self.command
    }
}

/// Proof that one bounded command passed explicit maintenance authority.
///
/// This is intentionally distinct from closed-loop `AuthorizedActuation`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MaintenanceActuation {
    command: BoundedActuatorCommand,
}

impl MaintenanceActuation {
    pub const fn command(self) -> BoundedActuatorCommand {
        self.command
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AuthorityOutcome {
    decision: AuthorityDecision,
    authorized: Option<AuthorizedActuation>,
}

impl AuthorityOutcome {
    pub const fn decision(self) -> AuthorityDecision {
        self.decision
    }

    pub const fn authorized(self) -> Option<AuthorizedActuation> {
        self.authorized
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AuthorityError {
    Busy,
    Faulted,
    NotMaintenance,
}

/// Supervisor-owned physical-output authority.
///
/// Closed-loop and maintenance authority are mutually exclusive runtime modes.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct RuntimeAuthority {
    mode: AuthorityMode,
}

impl RuntimeAuthority {
    pub const fn new() -> Self {
        Self {
            mode: AuthorityMode::Disarmed,
        }
    }

    pub const fn mode(self) -> AuthorityMode {
        self.mode
    }

    pub fn enter_closed_loop(&mut self) -> Result<(), AuthorityError> {
        self.enter(AuthorityMode::ClosedLoop)
    }

    pub fn enter_maintenance(&mut self) -> Result<(), AuthorityError> {
        self.enter(AuthorityMode::Maintenance)
    }

    pub fn release(&mut self) {
        if self.mode != AuthorityMode::Fault {
            self.mode = AuthorityMode::Disarmed;
        }
    }

    pub fn enter_fault(&mut self) {
        self.mode = AuthorityMode::Fault;
    }

    pub fn clear_fault(&mut self) {
        self.mode = AuthorityMode::Disarmed;
    }

    pub fn evaluate(
        self,
        context: AuthorityContext,
        command: BoundedActuatorCommand,
    ) -> AuthorityOutcome {
        let mut reasons = AuthorityReasons::NONE;
        let mut denied = false;
        let mut constrained = false;

        if self.mode != AuthorityMode::ClosedLoop {
            reasons = reasons.with(AuthorityReasons::MODE);
            denied = true;
        }
        if !matches!(context.runtime_state, RuntimeState::Active(_)) {
            reasons = reasons.with(AuthorityReasons::RUNTIME_STATE);
            denied = true;
        }
        if !context.timing.closed_loop_eligible() {
            reasons = reasons.with(AuthorityReasons::SENSOR_TIMING);
            denied = true;
        }
        if context.watchdog != WatchdogHealth::Healthy {
            reasons = reasons.with(AuthorityReasons::WATCHDOG);
            denied = true;
        }
        if context.estimate_validity != StateValidity::Valid {
            reasons = reasons.with(AuthorityReasons::ESTIMATE_INVALID);
            denied = true;
        }
        if !context.runtime_qualified {
            reasons = reasons.with(AuthorityReasons::RUNTIME_QUALIFICATION);
            denied = true;
        }
        if command.saturated {
            reasons = reasons.with(AuthorityReasons::ACTUATOR_SATURATED);
            constrained = true;
        }

        let decision = AuthorityDecision {
            authority: if denied {
                ActuationAuthority::Denied
            } else {
                ActuationAuthority::ClosedLoop
            },
            reasons,
            constrained,
        };

        AuthorityOutcome {
            decision,
            authorized: (!denied).then_some(AuthorizedActuation { command }),
        }
    }

    pub fn authorize_maintenance(
        self,
        command: BoundedActuatorCommand,
    ) -> Result<MaintenanceActuation, AuthorityError> {
        match self.mode {
            AuthorityMode::Maintenance => Ok(MaintenanceActuation { command }),
            AuthorityMode::Fault => Err(AuthorityError::Faulted),
            _ => Err(AuthorityError::NotMaintenance),
        }
    }

    fn enter(&mut self, target: AuthorityMode) -> Result<(), AuthorityError> {
        match self.mode {
            AuthorityMode::Disarmed => {
                self.mode = target;
                Ok(())
            }
            AuthorityMode::Fault => Err(AuthorityError::Faulted),
            _ => Err(AuthorityError::Busy),
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

#[cfg(test)]
mod tests {
    use super::*;
    use rip_actuator_model::BoundedActuatorCommand;
    use rip_robot_domain::{NormalizedCommand, TorqueNm};

    fn command(saturated: bool) -> BoundedActuatorCommand {
        BoundedActuatorCommand {
            command: NormalizedCommand::new(0.25).unwrap(),
            saturated,
            predicted_arm_torque: TorqueNm(0.05),
        }
    }

    fn healthy_context() -> AuthorityContext {
        AuthorityContext {
            runtime_state: RuntimeState::Active(ControlRegime::Balance),
            timing: SensorTimingHealth::Healthy,
            watchdog: WatchdogHealth::Healthy,
            estimate_validity: StateValidity::Valid,
            runtime_qualified: true,
        }
    }

    #[test]
    fn closed_loop_token_requires_closed_loop_mode_and_healthy_context() {
        let mut authority = RuntimeAuthority::new();
        assert!(authority
            .evaluate(healthy_context(), command(false))
            .authorized()
            .is_none());

        authority.enter_closed_loop().unwrap();
        assert!(authority
            .evaluate(healthy_context(), command(false))
            .authorized()
            .is_some());
    }

    #[test]
    fn actuator_saturation_is_explicitly_constrained_not_silently_dropped() {
        let mut authority = RuntimeAuthority::new();
        authority.enter_closed_loop().unwrap();
        let outcome = authority.evaluate(healthy_context(), command(true));

        assert!(outcome.authorized().is_some());
        assert!(outcome.decision().constrained);
        assert!(outcome
            .decision()
            .reasons
            .contains(AuthorityReasons::ACTUATOR_SATURATED));
    }

    #[test]
    fn maintenance_and_closed_loop_modes_are_mutually_exclusive() {
        let mut authority = RuntimeAuthority::new();
        authority.enter_maintenance().unwrap();
        assert_eq!(authority.enter_closed_loop(), Err(AuthorityError::Busy));
        assert!(authority.authorize_maintenance(command(false)).is_ok());
    }

    #[test]
    fn watchdog_expires_after_configured_timeout() {
        let mut watchdog = ControlWatchdog::new(5_000).unwrap();
        assert_eq!(watchdog.health(1_000), WatchdogHealth::Disarmed);
        watchdog.kick(1_000);
        assert_eq!(watchdog.health(5_000), WatchdogHealth::Healthy);
        assert_eq!(watchdog.health(6_001), WatchdogHealth::Expired);
    }
}
