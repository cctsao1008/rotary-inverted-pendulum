#![no_std]
#![forbid(unsafe_code)]

use rip_actuator_model::{ActuatorModelError, ArmActuatorModel, BoundedActuatorCommand};
use rip_robot_domain::{EstimatedState, GeneralizedDemand};
use rip_runtime_state::{
    AuthorityContext, AuthorityDecision, AuthorizedActuation, RuntimeAuthority, RuntimeLimits,
    RuntimePolicy, RuntimeQualification, RuntimeState, SensorTimingHealth, WatchdogHealth,
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
}

/// Deterministic portable control-runtime composition.
///
/// This runtime can produce an `AuthorizedActuation` proof token, but it has no
/// physical sink dependency. Firmware remains the only domain capable of
/// realizing electrical output.
pub struct ControlRuntime<S, C> {
    source: S,
    estimator: BasicEstimator,
    estimator_config: EstimatorConfig,
    runtime_limits: RuntimeLimits,
    controller: C,
    actuator_model: ArmActuatorModel,
    authority: RuntimeAuthority,
    runtime_state: RuntimeState,
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
            authority: RuntimeAuthority::new(),
            runtime_state: RuntimeState::Ready,
        }
    }

    pub const fn runtime_state(&self) -> RuntimeState {
        self.runtime_state
    }

    pub fn set_runtime_state(&mut self, state: RuntimeState) {
        self.runtime_state = state;
    }

    pub fn authority_mut(&mut self) -> &mut RuntimeAuthority {
        &mut self.authority
    }

    pub fn step(&mut self) -> Result<ControlCycle, CycleError<S::Error, C::Error>> {
        let observation = self.source.observe().map_err(CycleError::Source)?;
        let state = match self
            .estimator
            .step(self.estimator_config, observation.measurement)
            .map_err(CycleError::Estimator)?
        {
            Estimate::Primed => return Ok(ControlCycle::Primed),
            Estimate::Ready(state) => state,
        };

        let qualification = RuntimePolicy::qualify(
            Some(state),
            observation.sensor_valid,
            observation.sample_age_us,
            self.runtime_limits,
        );
        if !qualification.allowed {
            return Ok(ControlCycle::Rejected { qualification });
        }

        let demand = self
            .controller
            .compute(&state)
            .map_err(CycleError::Controller)?;
        let bounded_command = self
            .actuator_model
            .command_for_demand(demand)
            .map_err(CycleError::ActuatorModel)?;

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
}

#[cfg(test)]
mod tests {
    use super::*;
    use rip_actuator_model::ArmActuatorParameters;
    use rip_robot_domain::{AngleRad, TimestampUs};
    use rip_runtime_state::ActuationAuthority;
    use rip_state_feedback::LqrController;

    #[derive(Clone, Copy)]
    struct Source {
        samples: [RuntimeObservation; 2],
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

    fn sample(theta: f32, phi: f32, timestamp_us: u64) -> RuntimeObservation {
        RuntimeObservation {
            measurement: EstimatorMeasurement {
                theta: AngleRad(theta),
                phi: AngleRad(phi),
                captured_at: TimestampUs(timestamp_us),
            },
            sensor_valid: true,
            sample_age_us: 0,
            timing: SensorTimingHealth::Healthy,
            watchdog: WatchdogHealth::Healthy,
        }
    }

    fn runtime() -> ControlRuntime<Source, LqrController> {
        ControlRuntime::new(
            Source {
                samples: [sample(0.1, 0.0, 1_000), sample(0.11, 0.01, 11_000)],
                index: 0,
            },
            EstimatorConfig {
                max_gap_us: 20_000,
                rate_filter_alpha: 1.0,
            },
            RuntimeLimits::observe_only(),
            LqrController::new([0.1, 0.01, 0.1, 0.01]).unwrap(),
            ArmActuatorModel::new(ArmActuatorParameters::new(1.0, 0.0).unwrap()).unwrap(),
        )
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
    }
}
