use rip_control::{
    BasicEstimator, ControlEffort, ControlSafety, Controller, Estimate, EstimatorConfig,
    EstimatorError, EstimatorInput, FaultSet, SafetyLimits,
};

use crate::{Observation, SensorSource};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ObserveCycle {
    Primed,
    Rejected {
        faults: FaultSet,
    },
    Computed {
        state: rip_control::ControlState,
        effort: ControlEffort,
    },
}

#[derive(Debug, PartialEq)]
pub enum CycleError<SensorError, ControllerError> {
    Sensor(SensorError),
    Estimator(EstimatorError),
    Controller(ControllerError),
}

pub struct ObserveRuntime<S, C> {
    sensors: S,
    estimator: BasicEstimator,
    estimator_config: EstimatorConfig,
    safety_limits: SafetyLimits,
    controller: C,
}

impl<S, C> ObserveRuntime<S, C>
where
    S: SensorSource,
    C: Controller,
{
    pub fn new(
        sensors: S,
        estimator_config: EstimatorConfig,
        safety_limits: SafetyLimits,
        controller: C,
    ) -> Self {
        Self {
            sensors,
            estimator: BasicEstimator::new(),
            estimator_config,
            safety_limits,
            controller,
        }
    }

    pub fn step(&mut self) -> Result<ObserveCycle, CycleError<S::Error, C::Error>> {
        let observation = self.sensors.observe().map_err(CycleError::Sensor)?;
        let input = EstimatorInput::new(
            observation.theta_rad,
            observation.phi_rad,
            observation.timestamp_us,
        );

        let state = match self
            .estimator
            .step(&self.estimator_config, input)
            .map_err(CycleError::Estimator)?
        {
            Estimate::Primed => return Ok(ObserveCycle::Primed),
            Estimate::Ready(state) => state,
        };

        let safety = ControlSafety::check(
            Some(state),
            observation.valid,
            true,
            observation.sample_age_us,
            &self.safety_limits,
        );
        if !safety.allowed {
            return Ok(ObserveCycle::Rejected {
                faults: safety.faults,
            });
        }

        let effort = self
            .controller
            .compute(&state)
            .map_err(CycleError::Controller)?;
        Ok(ObserveCycle::Computed { state, effort })
    }
}
