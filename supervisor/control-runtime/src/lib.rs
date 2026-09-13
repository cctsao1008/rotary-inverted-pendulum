#![no_std]
#![forbid(unsafe_code)]

use rip_actuator_model::{ActuatorModelError, ArmActuatorModel, BoundedActuatorCommand};
use rip_actuator_safety::{
    CommandSafetyError, CommandSafetyGate, CommandSafetyOutcome, CommandSafetyProfile,
};
use rip_robot_domain::{EstimatedState, GeneralizedDemand};
use rip_runtime_state::{
    AdmissionContext, AdmissionDecision, AdmissionLimits, AuthorityContext, AuthorityDecision,
    AuthorizedActuation, ClosedLoopRequest, RunPermitContext, RunPermitDecision, RuntimeAuthority,
    RuntimeLimits, RuntimePolicy, RuntimeQualification, RuntimeState, SensorTimingHealth,
    WatchdogHealth,
};
use rip_state_estimator::{
    BasicEstimator, Estimate, EstimatorConfig, EstimatorError, EstimatorMeasurement,
};
use rip_state_feedback::Controller;

/// Supervisor-facing input contract produced by Firmware adapters.
///
/// Measurement semantics are distinct from runtime validity/timing evidence.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RuntimeObservation {
    pub measurement: EstimatorMeasurement,
    pub sensor_valid: bool,
    pub sample_age_us: u64,
    pub timing: SensorTimingHealth,
    pub watchdog: WatchdogHealth,
}

/// Consumer-owned port for one estimator/runtime observation opportunity.
pub trait RuntimeObservationSource {
    type Error;

    fn observe(&mut self) -> Result<RuntimeObservation, Self::Error>;
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ControlCycle {
    Primed,
    Rejected {
        qualification: RuntimeQualification,
    },
    Computed {
        state: EstimatedState,
        demand: GeneralizedDemand,
        bounded_command: BoundedActuatorCommand,
        authority: AuthorityDecision,
        authorized: Option<AuthorizedActuation>,
    },
}

#[derive(Debug, PartialEq)]
pub enum CycleError<SourceError, ControllerError> {
    Source(SourceError),
    Estimator(EstimatorError),
    Controller(ControllerError),
    ActuatorModel(ActuatorModelError),
    CommandSafety(CommandSafetyError),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClosedLoopRequestError {
    NotReady,
    AlreadyActive,
    Faulted,
    OutputSafetyUnconfigured,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommandSafetyConfigError {
    ClosedLoopActive,
    Faulted,
}

/// Deterministic portable control-runtime composition.
///
/// This runtime can produce an `AuthorizedActuation` proof token, but it has no
/// physical sink dependency. Firmware remains the only domain capable of
/// realizing electrical output.
///
/// Closed-loop authority is not directly mutable from outside this type. A
/// caller must submit an explicit `ClosedLoopRequest`; admission, output-safety,
/// and continuous run-permit checks then own the Ready -> Active transition and
/// any subsequent authority release.
pub struct ControlRuntime<S, C> {
    source: S,
    estimator: BasicEstimator,
    estimator_config: EstimatorConfig,
    runtime_limits: RuntimeLimits,
    controller: C,
    actuator_model: ArmActuatorModel,
    command_safety: CommandSafetyGate,
    authority: RuntimeAuthority,
    runtime_state: RuntimeState,
    admission_limits: Option<AdmissionLimits>,
    closed_loop_request: Option<ClosedLoopRequest>,
    last_admission: Option<AdmissionDecision>,
    last_run_permit: Option<RunPermitDecision>,
    last_command_safety: Option<CommandSafetyOutcome>,
}

impl<S, C> ControlRuntime<S, C>
where
    S: RuntimeObservationSource,
    C: Controller,
{
    pub fn new(
        source: S,
        estimator_config: EstimatorConfig,
        runtime_limits: RuntimeLimits,
        controller: C,
        actuator_model: ArmActuatorModel,
    ) -> Self {
        Self {
            source,
            estimator: BasicEstimator::new(),
            estimator_config,
            runtime_limits,
            controller,
            actuator_model,
            command_safety: CommandSafetyGate::new(),
            authority: RuntimeAuthority::new(),
            runtime_state: RuntimeState::Ready,
            admission_limits: None,
            closed_loop_request: None,
            last_admission: None,
            last_run_permit: None,
            last_command_safety: None,
        }
    }

    pub const fn runtime_state(&self) -> RuntimeState {
        self.runtime_state
    }

    pub const fn closed_loop_requested(&self) -> bool {
        self.closed_loop_request.is_some()
    }

    pub const fn last_admission_decision(&self) -> Option<AdmissionDecision> {
        self.last_admission
    }

    pub const fn last_run_permit_decision(&self) -> Option<RunPermitDecision> {
        self.last_run_permit
    }

    pub const fn last_command_safety_outcome(&self) -> Option<CommandSafetyOutcome> {
        self.last_command_safety
    }

    pub const fn command_safety_configured(&self) -> bool {
        self.command_safety.is_configured()
    }

    /// Configure the admission-only near-upright boundary.
    ///
    /// Absence is deliberately fail-closed for every closed-loop request. The
    /// boundary is evaluated only at admission; it is not reused as a continuous
    /// run-permit condition.
    pub fn configure_admission_limits(&mut self, limits: AdmissionLimits) {
        self.admission_limits = Some(limits);
    }

    /// Configure the independent output-safety profile used by automatic control.
    ///
    /// Reconfiguration is forbidden while closed-loop authority is active. A
    /// successful configuration resets slew history to zero authority.
    pub fn configure_command_safety(
        &mut self,
        profile: CommandSafetyProfile,
    ) -> Result<(), CommandSafetyConfigError> {
        match self.runtime_state {
            RuntimeState::Active(_) => Err(CommandSafetyConfigError::ClosedLoopActive),
            RuntimeState::Fault(_) => Err(CommandSafetyConfigError::Faulted),
            RuntimeState::Ready | RuntimeState::Disabled => {
                self.command_safety.configure(profile);
                self.last_command_safety = None;
                Ok(())
            }
        }
    }

    /// Record explicit operator/supervisor intent to enter closed loop.
    ///
    /// A request does not itself change runtime state or physical authority. It
    /// remains pending while admission conditions are temporarily false. Output
    /// safety must already be explicitly configured before a request is accepted.
    pub fn request_closed_loop(
        &mut self,
        request: ClosedLoopRequest,
    ) -> Result<(), ClosedLoopRequestError> {
        match self.runtime_state {
            RuntimeState::Ready => {
                if !self.command_safety.is_configured() {
                    return Err(ClosedLoopRequestError::OutputSafetyUnconfigured);
                }
                self.closed_loop_request = Some(request);
                Ok(())
            }
            RuntimeState::Active(_) => Err(ClosedLoopRequestError::AlreadyActive),
            RuntimeState::Fault(_) => Err(ClosedLoopRequestError::Faulted),
            RuntimeState::Disabled => Err(ClosedLoopRequestError::NotReady),
        }
    }

    /// Explicitly cancel pending/active closed-loop intent.
    pub fn cancel_closed_loop(&mut self) {
        self.closed_loop_request = None;
        self.last_admission = None;
        self.last_run_permit = None;
        self.last_command_safety = None;
        self.release_closed_loop_authority();
    }

    /// Enter a fail-closed disabled state and release non-fault authority.
    pub fn disable(&mut self) {
        self.cancel_closed_loop();
        self.runtime_state = RuntimeState::Disabled;
    }

    /// Return from Disabled to the non-actuating admission-ready state.
    pub fn enable_ready(&mut self) -> bool {
        if self.runtime_state == RuntimeState::Disabled {
            self.runtime_state = RuntimeState::Ready;
            true
        } else {
            false
        }
    }

    /// Mutable access to the consumer-owned observation source.
    ///
    /// Firmware targets use this to submit one freshly acquired observation
    /// without taking ownership of estimator, controller, actuator-model, or
    /// authority semantics.
    pub fn source_mut(&mut self) -> &mut S {
        &mut self.source
    }

    /// Read-only access to controller state for non-authoritative telemetry.
    pub const fn controller(&self) -> &C {
        &self.controller
    }

    pub fn step(&mut self) -> Result<ControlCycle, CycleError<S::Error, C::Error>> {
        self.last_command_safety = None;
        let observation = self.source.observe().map_err(CycleError::Source)?;
        let state = match self
            .estimator
            .step(self.estimator_config, observation.measurement)
            .map_err(CycleError::Estimator)?
        {
            Estimate::Primed => {
                // A re-prime means no fresh derivative/state is available for
                // continuous control. Active authority must not survive it.
                if matches!(self.runtime_state, RuntimeState::Active(_)) {
                    self.release_closed_loop_after_runtime_loss();
                }
                return Ok(ControlCycle::Primed);
            }
            Estimate::Ready(state) => state,
        };

        let qualification = RuntimePolicy::qualify(
            Some(state),
            observation.sensor_valid,
            observation.sample_age_us,
            self.runtime_limits,
        );

        self.update_closed_loop_state(state, observation, qualification);

        if !qualification.allowed {
            return Ok(ControlCycle::Rejected { qualification });
        }

        let demand = self
            .controller
            .compute(&state)
            .map_err(CycleError::Controller)?;
        let mapped_command = self
            .actuator_model
            .command_for_demand(demand)
            .map_err(CycleError::ActuatorModel)?;

        let bounded_command = if matches!(self.runtime_state, RuntimeState::Active(_)) {
            match self.command_safety.constrain_mapped_command(
                self.actuator_model,
                mapped_command,
                state.timestamp,
            ) {
                Ok(safe) => {
                    self.last_command_safety = Some(safe.safety);
                    safe.bounded_command
                }
                Err(error) => {
                    self.release_closed_loop_after_runtime_loss();
                    return Err(CycleError::CommandSafety(error));
                }
            }
        } else {
            mapped_command
        };

        let outcome = self.authority.evaluate(
            AuthorityContext {
                runtime_state: self.runtime_state,
                timing: observation.timing,
                watchdog: observation.watchdog,
                estimate_validity: state.validity,
                runtime_qualified: qualification.allowed,
            },
            bounded_command,
        );

        Ok(ControlCycle::Computed {
            state,
            demand,
            bounded_command,
            authority: outcome.decision(),
            authorized: outcome.authorized(),
        })
    }

    fn update_closed_loop_state(
        &mut self,
        state: EstimatedState,
        observation: RuntimeObservation,
        qualification: RuntimeQualification,
    ) {
        self.last_admission = None;
        self.last_run_permit = None;

        if matches!(self.runtime_state, RuntimeState::Active(_)) {
            let permit = RuntimePolicy::run_permit(RunPermitContext {
                runtime_state: self.runtime_state,
                timing: observation.timing,
                watchdog: observation.watchdog,
                estimate_validity: state.validity,
                runtime_qualified: qualification.allowed,
                authority_mode: self.authority.mode(),
            });
            self.last_run_permit = Some(permit);
            if !permit.allowed {
                self.release_closed_loop_after_runtime_loss();
            }
            return;
        }

        if self.runtime_state != RuntimeState::Ready {
            return;
        }

        let Some(request) = self.closed_loop_request else {
            return;
        };

        let admission = RuntimePolicy::admit(
            request,
            state,
            AdmissionContext {
                runtime_state: self.runtime_state,
                timing: observation.timing,
                watchdog: observation.watchdog,
                estimate_validity: state.validity,
                runtime_qualified: qualification.allowed,
                authority_mode: self.authority.mode(),
            },
            self.admission_limits,
        );
        self.last_admission = Some(admission);

        if !admission.allowed {
            return;
        }

        // Admission already proved Disarmed authority, so this transition is
        // deterministic and cannot race inside the single-owner runtime.
        if self.authority.enter_closed_loop().is_err() {
            return;
        }
        self.runtime_state = RuntimeState::Active(request.regime());

        let permit = RuntimePolicy::run_permit(RunPermitContext {
            runtime_state: self.runtime_state,
            timing: observation.timing,
            watchdog: observation.watchdog,
            estimate_validity: state.validity,
            runtime_qualified: qualification.allowed,
            authority_mode: self.authority.mode(),
        });
        self.last_run_permit = Some(permit);
        if !permit.allowed {
            self.release_closed_loop_after_runtime_loss();
        }
    }

    fn release_closed_loop_after_runtime_loss(&mut self) {
        self.closed_loop_request = None;
        self.release_closed_loop_authority();
    }

    fn release_closed_loop_authority(&mut self) {
        self.authority.release();
        self.command_safety.reset_history();
        self.last_command_safety = None;
        if matches!(self.runtime_state, RuntimeState::Active(_)) {
            self.runtime_state = RuntimeState::Ready;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rip_actuator_model::ArmActuatorParameters;
    use rip_actuator_safety::{CommandConstraintReasons, CommandSafetyLimits, SafetyProfileKind};
    use rip_hybrid_control::ControlRegime;
    use rip_robot_domain::{AngleRad, NormalizedCommand, TimestampUs};
    use rip_runtime_state::ActuationAuthority;
    use rip_state_feedback::LqrController;

    #[derive(Clone, Copy)]
    struct Source {
        samples: [RuntimeObservation; 5],
        index: usize,
    }

    impl RuntimeObservationSource for Source {
        type Error = ();

        fn observe(&mut self) -> Result<RuntimeObservation, Self::Error> {
            let sample = self.samples[self.index.min(self.samples.len() - 1)];
            self.index += 1;
            Ok(sample)
        }
    }

    fn sample(
        theta: f32,
        phi: f32,
        timestamp_us: u64,
        timing: SensorTimingHealth,
        watchdog: WatchdogHealth,
    ) -> RuntimeObservation {
        RuntimeObservation {
            measurement: EstimatorMeasurement {
                theta: AngleRad(theta),
                phi: AngleRad(phi),
                captured_at: TimestampUs(timestamp_us),
            },
            sensor_valid: true,
            sample_age_us: 0,
            timing,
            watchdog,
        }
    }

    fn healthy_sample(theta: f32, phi: f32, timestamp_us: u64) -> RuntimeObservation {
        sample(
            theta,
            phi,
            timestamp_us,
            SensorTimingHealth::Healthy,
            WatchdogHealth::Healthy,
        )
    }

    fn runtime_with_samples(
        samples: [RuntimeObservation; 5],
    ) -> ControlRuntime<Source, LqrController> {
        ControlRuntime::new(
            Source { samples, index: 0 },
            EstimatorConfig {
                max_gap_us: 20_000,
                rate_filter_alpha: 1.0,
            },
            RuntimeLimits::observe_only(),
            LqrController::new([0.1, 0.01, 0.1, 0.01]).unwrap(),
            ArmActuatorModel::new(ArmActuatorParameters::new(1.0, 0.0).unwrap()).unwrap(),
        )
    }

    fn runtime() -> ControlRuntime<Source, LqrController> {
        runtime_with_samples([
            healthy_sample(0.10, 0.00, 1_000),
            healthy_sample(0.11, 0.01, 11_000),
            healthy_sample(0.12, 0.02, 21_000),
            healthy_sample(0.13, 0.03, 31_000),
            healthy_sample(0.14, 0.04, 41_000),
        ])
    }

    fn configure_admission(runtime: &mut ControlRuntime<Source, LqrController>) {
        runtime.configure_admission_limits(AdmissionLimits::new(0.20).unwrap());
    }

    fn configure_output_safety(runtime: &mut ControlRuntime<Source, LqrController>) {
        let limits = CommandSafetyLimits::new(1.0, 200.0).unwrap();
        runtime
            .configure_command_safety(CommandSafetyProfile::new(
                SafetyProfileKind::Simulation,
                limits,
            ))
            .unwrap();
    }

    #[test]
    fn default_runtime_computes_without_granting_physical_authority() {
        let mut runtime = runtime();
        assert_eq!(runtime.step().unwrap(), ControlCycle::Primed);

        match runtime.step().unwrap() {
            ControlCycle::Computed {
                authority,
                authorized,
                ..
            } => {
                assert_eq!(authority.authority, ActuationAuthority::Denied);
                assert!(authorized.is_none());
            }
            _ => panic!("second observation must compute"),
        }
        assert!(runtime.last_command_safety_outcome().is_none());
    }

    #[test]
    fn closed_loop_request_requires_output_safety_configuration() {
        let mut runtime = runtime();
        configure_admission(&mut runtime);

        assert_eq!(
            runtime.request_closed_loop(ClosedLoopRequest::new(ControlRegime::Balance)),
            Err(ClosedLoopRequestError::OutputSafetyUnconfigured)
        );
        assert!(!runtime.closed_loop_requested());
    }

    #[test]
    fn explicit_request_and_admission_are_required_before_authority() {
        let mut runtime = runtime();
        configure_admission(&mut runtime);
        configure_output_safety(&mut runtime);
        runtime
            .request_closed_loop(ClosedLoopRequest::new(ControlRegime::Balance))
            .unwrap();

        assert_eq!(runtime.step().unwrap(), ControlCycle::Primed);
        match runtime.step().unwrap() {
            ControlCycle::Computed {
                bounded_command,
                authorized,
                ..
            } => {
                assert!(authorized.is_some());
                // First automatic command after zero-authority reset must earn
                // output magnitude through the slew limiter.
                assert_eq!(bounded_command.command, NormalizedCommand::ZERO);
            }
            _ => panic!("admitted second observation must compute"),
        }
        assert_eq!(
            runtime.runtime_state(),
            RuntimeState::Active(ControlRegime::Balance)
        );
        assert!(runtime.last_admission_decision().unwrap().allowed);
        assert!(runtime.last_run_permit_decision().unwrap().allowed);
        assert!(runtime
            .last_command_safety_outcome()
            .unwrap()
            .reasons
            .contains(CommandConstraintReasons::SLEW));
    }

    #[test]
    fn admission_configuration_is_fail_closed() {
        let mut runtime = runtime();
        configure_output_safety(&mut runtime);
        runtime
            .request_closed_loop(ClosedLoopRequest::new(ControlRegime::Balance))
            .unwrap();
        assert_eq!(runtime.step().unwrap(), ControlCycle::Primed);
        match runtime.step().unwrap() {
            ControlCycle::Computed { authorized, .. } => assert!(authorized.is_none()),
            _ => panic!("second observation must compute shadow output"),
        }
        assert_eq!(runtime.runtime_state(), RuntimeState::Ready);
        assert!(!runtime.last_admission_decision().unwrap().allowed);
    }

    #[test]
    fn output_safety_cannot_be_reconfigured_while_active() {
        let mut runtime = runtime();
        configure_admission(&mut runtime);
        configure_output_safety(&mut runtime);
        runtime
            .request_closed_loop(ClosedLoopRequest::new(ControlRegime::Balance))
            .unwrap();
        assert_eq!(runtime.step().unwrap(), ControlCycle::Primed);
        assert!(matches!(
            runtime.step().unwrap(),
            ControlCycle::Computed { .. }
        ));

        let replacement = CommandSafetyProfile::new(
            SafetyProfileKind::Simulation,
            CommandSafetyLimits::new(0.5, 10.0).unwrap(),
        );
        assert_eq!(
            runtime.configure_command_safety(replacement),
            Err(CommandSafetyConfigError::ClosedLoopActive)
        );
    }

    #[test]
    fn run_permit_loss_clears_intent_and_restarts_slew_history() {
        let mut runtime = runtime_with_samples([
            healthy_sample(0.10, 0.00, 1_000),
            healthy_sample(0.11, 0.01, 11_000),
            sample(
                0.12,
                0.02,
                21_000,
                SensorTimingHealth::Timeout,
                WatchdogHealth::Healthy,
            ),
            healthy_sample(0.13, 0.03, 31_000),
            healthy_sample(0.14, 0.04, 41_000),
        ]);
        configure_admission(&mut runtime);
        configure_output_safety(&mut runtime);
        runtime
            .request_closed_loop(ClosedLoopRequest::new(ControlRegime::Balance))
            .unwrap();

        assert_eq!(runtime.step().unwrap(), ControlCycle::Primed);
        assert!(matches!(
            runtime.step().unwrap(),
            ControlCycle::Computed {
                authorized: Some(_),
                ..
            }
        ));
        assert!(matches!(
            runtime.step().unwrap(),
            ControlCycle::Computed {
                authorized: None,
                ..
            }
        ));
        assert_eq!(runtime.runtime_state(), RuntimeState::Ready);
        assert!(!runtime.closed_loop_requested());
        assert!(!runtime.last_run_permit_decision().unwrap().allowed);

        // Healthy data alone cannot silently restart closed loop.
        assert!(matches!(
            runtime.step().unwrap(),
            ControlCycle::Computed {
                authorized: None,
                ..
            }
        ));
        runtime
            .request_closed_loop(ClosedLoopRequest::new(ControlRegime::Balance))
            .unwrap();
        match runtime.step().unwrap() {
            ControlCycle::Computed {
                bounded_command,
                authorized,
                ..
            } => {
                assert!(authorized.is_some());
                assert_eq!(bounded_command.command, NormalizedCommand::ZERO);
            }
            _ => panic!("fresh request must re-enter through output safety"),
        }
    }

    #[test]
    fn estimator_reprime_releases_authority_and_clears_intent() {
        let mut runtime = runtime_with_samples([
            healthy_sample(0.10, 0.00, 1_000),
            healthy_sample(0.11, 0.01, 11_000),
            // 39 ms from the previous measurement exceeds the 20 ms estimator gap.
            healthy_sample(0.12, 0.02, 50_000),
            healthy_sample(0.13, 0.03, 60_000),
            healthy_sample(0.14, 0.04, 70_000),
        ]);
        configure_admission(&mut runtime);
        configure_output_safety(&mut runtime);
        runtime
            .request_closed_loop(ClosedLoopRequest::new(ControlRegime::Balance))
            .unwrap();

        assert_eq!(runtime.step().unwrap(), ControlCycle::Primed);
        assert!(matches!(
            runtime.step().unwrap(),
            ControlCycle::Computed {
                authorized: Some(_),
                ..
            }
        ));
        assert_eq!(runtime.step().unwrap(), ControlCycle::Primed);
        assert_eq!(runtime.runtime_state(), RuntimeState::Ready);
        assert!(!runtime.closed_loop_requested());
        assert!(runtime.last_command_safety_outcome().is_none());

        // Recovered estimator data does not silently restore old authority.
        assert!(matches!(
            runtime.step().unwrap(),
            ControlCycle::Computed {
                authorized: None,
                ..
            }
        ));
    }

    #[test]
    fn disable_requires_explicit_return_to_ready() {
        let mut runtime = runtime();
        runtime.disable();
        assert_eq!(runtime.runtime_state(), RuntimeState::Disabled);
        assert_eq!(
            runtime.request_closed_loop(ClosedLoopRequest::new(ControlRegime::Balance)),
            Err(ClosedLoopRequestError::NotReady)
        );
        assert!(runtime.enable_ready());
        assert_eq!(runtime.runtime_state(), RuntimeState::Ready);
    }

    #[test]
    fn observation_source_can_be_updated_without_exposing_runtime_internals() {
        let mut runtime = runtime();
        runtime.source_mut().index = 1;

        assert_eq!(runtime.source.index, 1);
    }

    #[test]
    fn controller_state_is_available_read_only() {
        let runtime = runtime();
        assert_eq!(runtime.controller().gains(), [0.1, 0.01, 0.1, 0.01]);
    }
}
