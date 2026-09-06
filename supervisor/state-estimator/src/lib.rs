#![no_std]
#![forbid(unsafe_code)]

use core::f32::consts::PI;
use rip_robot_domain::{
    AngleRad, AngularRateRadPerSec, EstimatedState, StateValidity, TimestampUs,
};

/// Supervisor-owned estimator input representation carrying Plant measurement semantics.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EstimatorMeasurement {
    pub theta: AngleRad,
    pub phi: AngleRad,
    pub captured_at: TimestampUs,
}

impl EstimatorMeasurement {
    pub fn is_finite(self) -> bool {
        self.theta.0.is_finite() && self.phi.0.is_finite()
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EstimatorConfig {
    pub max_gap_us: u64,
    pub rate_filter_alpha: f32,
}

impl EstimatorConfig {
    pub fn is_valid(self) -> bool {
        self.max_gap_us > 0
            && self.rate_filter_alpha.is_finite()
            && (0.0..=1.0).contains(&self.rate_filter_alpha)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EstimatorError {
    InvalidConfig,
    NonFiniteMeasurement,
    InvalidTimestamp,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Estimate {
    Primed,
    Ready(EstimatedState),
}

#[derive(Debug, Clone, Copy)]
pub struct BasicEstimator {
    previous: Option<EstimatorMeasurement>,
    theta_dot: AngularRateRadPerSec,
    phi_dot: AngularRateRadPerSec,
}

impl BasicEstimator {
    pub const fn new() -> Self {
        Self {
            previous: None,
            theta_dot: AngularRateRadPerSec(0.0),
            phi_dot: AngularRateRadPerSec(0.0),
        }
    }

    pub fn reset(&mut self) {
        *self = Self::new();
    }

    pub fn step(
        &mut self,
        config: EstimatorConfig,
        measurement: EstimatorMeasurement,
    ) -> Result<Estimate, EstimatorError> {
        if !config.is_valid() {
            return Err(EstimatorError::InvalidConfig);
        }
        if !measurement.is_finite() {
            return Err(EstimatorError::NonFiniteMeasurement);
        }

        let previous = match self.previous {
            Some(previous) => previous,
            None => {
                self.prime(measurement);
                return Ok(Estimate::Primed);
            }
        };

        if measurement.captured_at <= previous.captured_at {
            return Err(EstimatorError::InvalidTimestamp);
        }

        let elapsed_us = measurement.captured_at.0 - previous.captured_at.0;
        if elapsed_us > config.max_gap_us {
            self.prime(measurement);
            return Ok(Estimate::Primed);
        }

        let dt_s = elapsed_us as f32 * 1.0e-6;
        let theta_dot_raw = shortest_circular_delta(measurement.theta.0 - previous.theta.0) / dt_s;
        let phi_dot_raw = (measurement.phi.0 - previous.phi.0) / dt_s;
        if !theta_dot_raw.is_finite() || !phi_dot_raw.is_finite() {
            return Err(EstimatorError::NonFiniteMeasurement);
        }

        let alpha = config.rate_filter_alpha;
        self.theta_dot =
            AngularRateRadPerSec(alpha * theta_dot_raw + (1.0 - alpha) * self.theta_dot.0);
        self.phi_dot = AngularRateRadPerSec(alpha * phi_dot_raw + (1.0 - alpha) * self.phi_dot.0);
        self.previous = Some(measurement);

        Ok(Estimate::Ready(EstimatedState {
            timestamp: measurement.captured_at,
            theta: measurement.theta,
            theta_dot: self.theta_dot,
            phi: measurement.phi,
            phi_dot: self.phi_dot,
            validity: StateValidity::Valid,
        }))
    }

    fn prime(&mut self, measurement: EstimatorMeasurement) {
        self.previous = Some(measurement);
        self.theta_dot = AngularRateRadPerSec(0.0);
        self.phi_dot = AngularRateRadPerSec(0.0);
    }
}

impl Default for BasicEstimator {
    fn default() -> Self {
        Self::new()
    }
}

fn shortest_circular_delta(mut delta: f32) -> f32 {
    let tau = 2.0 * PI;
    if delta > PI {
        delta -= tau;
    } else if delta < -PI {
        delta += tau;
    }
    delta
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config() -> EstimatorConfig {
        EstimatorConfig {
            max_gap_us: 20_000,
            rate_filter_alpha: 1.0,
        }
    }

    fn measurement(theta: f32, phi: f32, timestamp_us: u64) -> EstimatorMeasurement {
        EstimatorMeasurement {
            theta: AngleRad(theta),
            phi: AngleRad(phi),
            captured_at: TimestampUs(timestamp_us),
        }
    }

    #[test]
    fn first_measurement_primes_history() {
        let mut estimator = BasicEstimator::new();
        assert_eq!(
            estimator
                .step(config(), measurement(0.1, 0.2, 1_000))
                .unwrap(),
            Estimate::Primed
        );
    }

    #[test]
    fn theta_rate_uses_shortest_circular_delta() {
        let mut estimator = BasicEstimator::new();
        estimator
            .step(config(), measurement(3.13, 0.0, 1_000))
            .unwrap();

        match estimator
            .step(config(), measurement(-3.13, 0.0, 11_000))
            .unwrap()
        {
            Estimate::Ready(state) => assert!(state.theta_dot.0.abs() < 10.0),
            Estimate::Primed => panic!("second measurement must produce an estimate"),
        }
    }
}
